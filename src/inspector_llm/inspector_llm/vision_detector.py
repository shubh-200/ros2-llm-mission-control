"""
Vision Target Detector - ROS 2 Node

Subscribes to /camera/image (RGB) and /camera/points (PointCloud2),
detects colored objects via HSV segmentation, computes their 3D position
from the depth data, and broadcasts a TF transform + PoseStamped topic.

SWAPPING DETECTION BACKEND:
──────────────────────────
This node uses HSV color segmentation by default (fast, works well in Gazebo).
To swap to a different detection method, replace the `_detect_target()` method:

  1. YOLOv8:
     - pip install ultralytics
     - Load model: self._yolo = YOLO('yolov8n.pt')
     - In _detect_target(): results = self._yolo(cv_image)
       For each result.boxes: get class name, bounding box center (u,v)
     - Return (center_u, center_v, confidence) or None

  2. Gemini Vision (VLM):
     - Use google-genai with image input
     - Encode frame as JPEG, send to Gemini with prompt:
       "Is there a {target} in this image? If yes, return bounding box [x1,y1,x2,y2]"
     - Parse response, compute center
     - CAVEAT: ~2s latency per frame - use only for one-shot ID, not continuous tracking
     - Hybrid approach: VLM identifies target once -> switch to HSV/color tracking

  3. Custom CNN:
     - Load ONNX model via cv2.dnn.readNetFromONNX()
     - Run inference in _detect_target()
"""

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CompressedImage
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import String
import cv2
import numpy as np
from cv_bridge import CvBridge
import struct
from tf2_ros import TransformBroadcaster
from datetime import datetime


# HSV color ranges for known targets
# These are tuned for Gazebo's rendering. Real-world would need calibration.
# Format: {name: {"lower": [H, S, V], "upper": [H, S, V]}}
TARGET_COLORS = {
    "red_target": {
        # Strict Hue selection [0-5] and [175-180] completely excludes brown (Hue 10-25)
        # Moderate Saturation/Value lets us track under shadows and lighting changes
        "ranges": [
            {"lower": [0, 120, 100], "upper": [5, 255, 255]},
            {"lower": [175, 120, 100], "upper": [180, 255, 255]},
        ],
        "label": "Red Target Box",
    },
    "cargo_box": {
        "ranges": [
            {"lower": [15, 80, 80], "upper": [30, 255, 200]},
        ],
        "label": "Cargo Box (Brown)",
    },
    "blue_barrel": {
        "ranges": [
            {"lower": [100, 100, 100], "upper": [130, 255, 255]},
        ],
        "label": "Blue Barrel",
    },
}

# Detection thresholds
MIN_CONTOUR_AREA = 1000    # pixels^2 - ignore background noise and small blobs
LOST_TIMEOUT_SEC = 2.0    # seconds without detection before publishing "lost"


class VisionDetector(Node):
    """
    Detects colored targets in the camera feed and publishes their 3D pose.

    Published topics:
      /detected_target   (PoseStamped)  - 3D pose in camera_link frame
      /detection_image   (Image)        - annotated camera feed for RViz
      /detection_status  (String)       - "tracking" | "lost" | "idle"

    Parameters:
      target_name (str): Key into TARGET_COLORS dict. Default: "red_target"
    """

    def __init__(self):
        super().__init__('vision_detector')

        # Parameters
        self.declare_parameter('target_name', 'red_target')
        self.declare_parameter('snapshot_dir', 'detections')
        self._target_name = self.get_parameter('target_name').value
        self._snapshot_dir = os.path.abspath(self.get_parameter('snapshot_dir').value)

        if self._target_name not in TARGET_COLORS:
            self.get_logger().error(
                f'Unknown target: {self._target_name}. '
                f'Available: {list(TARGET_COLORS.keys())}'
            )
            raise ValueError(f'Unknown target: {self._target_name}')

        self._target_cfg = TARGET_COLORS[self._target_name]
        self.get_logger().info(
            f'Vision detector initialized for: {self._target_cfg["label"]}'
        )

        # Ensure snapshot directory exists
        os.makedirs(self._snapshot_dir, exist_ok=True)
        self.get_logger().info(f'Detection snapshots will be saved to: {self._snapshot_dir}')

        # State
        self._bridge = CvBridge()
        self._latest_pc = None
        self._last_detection_time = None
        self._detection_active = True
        self._snapshot_saved = False   # only save once per mission (first detection)

        # Subscribers
        self.create_subscription(Image, '/camera/image', self._image_cb, 10)
        self.create_subscription(PointCloud2, '/camera/points', self._pc_cb, 10)

        # Publishers
        self._pose_pub = self.create_publisher(PoseStamped, '/detected_target', 10)
        self._img_pub = self.create_publisher(Image, '/detection_image', 10)
        self._status_pub = self.create_publisher(String, '/detection_status', 10)
        # CompressedImage snapshot pushed once on first detection
        self._snapshot_pub = self.create_publisher(CompressedImage, '/detection_snapshot', 1)

        # TF Broadcaster
        self._tf_broadcaster = TransformBroadcaster(self)

        # Timer for status checking
        self.create_timer(0.5, self._status_timer_cb)

        self.get_logger().info('Vision detector node started.')

    def _pc_cb(self, msg: PointCloud2):
        """Cache the latest point cloud for depth lookup."""
        self._latest_pc = msg

    def _image_cb(self, msg: Image):
        """Main detection callback - runs on every camera frame."""
        if not self._detection_active:
            return

        # Convert ROS Image -> OpenCV BGR
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        # Detect target
        result = self._detect_target(cv_image)

        if result is not None:
            center_u, center_v, bbox, mask_area = result
            self._last_detection_time = self.get_clock().now()

            # Draw detection overlay
            annotated = cv_image.copy()
            x, y, w, h = bbox
            cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(annotated, f'{self._target_cfg["label"]} ({mask_area}px)',
                        (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.circle(annotated, (center_u, center_v), 5, (0, 0, 255), -1)

            # First-detection operator notification
            if not self._snapshot_saved:
                self._save_snapshot(annotated)
                self._snapshot_saved = True

            # Publish annotated image (continuous stream)
            self._img_pub.publish(
                self._bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            )

            # 3D localization from point cloud
            if self._latest_pc is not None:
                pose_3d = self._lookup_depth(center_u, center_v, self._latest_pc)
                if pose_3d is not None:
                    px, py, pz = pose_3d

                    # Publish PoseStamped
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = self.get_clock().now().to_msg()
                    pose_msg.header.frame_id = 'camera_link'
                    pose_msg.pose.position.x = float(px)
                    pose_msg.pose.position.y = float(py)
                    pose_msg.pose.position.z = float(pz)
                    pose_msg.pose.orientation.w = 1.0
                    self._pose_pub.publish(pose_msg)

                    # Broadcast TF
                    t = TransformStamped()
                    t.header.stamp = self.get_clock().now().to_msg()
                    t.header.frame_id = 'camera_link'
                    t.child_frame_id = 'detected_target'
                    t.transform.translation.x = float(px)
                    t.transform.translation.y = float(py)
                    t.transform.translation.z = float(pz)
                    t.transform.rotation.w = 1.0
                    self._tf_broadcaster.sendTransform(t)

        else:
            # No detection - still publish annotated image (no overlay)
            self._img_pub.publish(
                self._bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            )

    def _detect_target(self, cv_image: np.ndarray):
        """
        HSV color segmentation detection.

        Returns (center_u, center_v, bbox, mask_area) or None.

        TO SWAP TO YOLO:
          from ultralytics import YOLO
          model = YOLO('yolov8n.pt')
          results = model(cv_image, verbose=False)
          for box in results[0].boxes:
              cls_name = results[0].names[int(box.cls)]
              if cls_name == self._target_name:
                  x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                  center_u = int((x1 + x2) / 2)
                  center_v = int((y1 + y2) / 2)
                  return (center_u, center_v, (int(x1), int(y1), int(x2-x1), int(y2-y1)), int((x2-x1)*(y2-y1)))
        """
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Combine all HSV ranges for this target
        combined_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for r in self._target_cfg["ranges"]:
            lower = np.array(r["lower"])
            upper = np.array(r["upper"])
            mask = cv2.inRange(hsv, lower, upper)
            combined_mask = cv2.bitwise_or(combined_mask, mask)

        # Morphological cleanup
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)

        # Find contours
        contours, _ = cv2.findContours(
            combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:
            return None

        # Pick largest contour above threshold
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)

        if area < MIN_CONTOUR_AREA:
            return None

        # Bounding box and center
        x, y, w, h = cv2.boundingRect(largest)
        center_u = x + w // 2
        center_v = y + h // 2

        return (center_u, center_v, (x, y, w, h), int(area))

    def _save_snapshot(self, annotated_frame: np.ndarray):
        """
        Save annotated detection frame to disk and publish a CompressedImage
        to /detection_snapshot so any subscriber (e.g. RViz, a web bridge,
        a logging service) receives the first-detection alert.
        """
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'detection_{self._target_name}_{timestamp}.jpg'
        filepath = os.path.join(self._snapshot_dir, filename)

        # Write to disk (visible on the host via the bind-mounted volume)
        success = cv2.imwrite(filepath, annotated_frame)
        if success:
            self.get_logger().info(
                f'[OPERATOR ALERT] Target first detected! Snapshot saved to: {filepath}'
            )
        else:
            self.get_logger().warn(f'Failed to write snapshot to {filepath}')

        # Also publish as CompressedImage for any ROS subscriber
        ret, jpeg_buf = cv2.imencode('.jpg', annotated_frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if ret:
            snapshot_msg = CompressedImage()
            snapshot_msg.header.stamp = self.get_clock().now().to_msg()
            snapshot_msg.format = 'jpeg'
            snapshot_msg.data = jpeg_buf.tobytes()
            self._snapshot_pub.publish(snapshot_msg)
            self.get_logger().info(
                f'[OPERATOR ALERT] Snapshot published to /detection_snapshot'
            )

    def _lookup_depth(self, u: int, v: int, pc_msg: PointCloud2):
        """
        Extract 3D point from organized PointCloud2 at pixel (u, v).

        Returns (x, y, z) in the point cloud's frame, or None if invalid.
        """
        width = pc_msg.width
        height = pc_msg.height

        if u < 0 or u >= width or v < 0 or v >= height:
            return None

        # Calculate byte offset into the point cloud data
        point_step = pc_msg.point_step
        row_step = pc_msg.row_step
        offset = v * row_step + u * point_step

        # Extract x, y, z (first 12 bytes, 3 floats)
        try:
            x, y, z = struct.unpack_from('fff', pc_msg.data, offset)
        except struct.error:
            return None

        # Filter NaN
        if np.isnan(x) or np.isnan(y) or np.isnan(z):
            return None

        # Filter unreasonable depths (>10m or <0.1m)
        depth = np.sqrt(x * x + y * y + z * z)
        if depth < 0.1 or depth > 10.0:
            return None

        return (x, y, z)

    def _status_timer_cb(self):
        """Publish detection status for the follow controller."""
        status = String()

        if self._last_detection_time is None:
            status.data = 'idle'
        else:
            elapsed = (self.get_clock().now() - self._last_detection_time).nanoseconds / 1e9
            if elapsed < LOST_TIMEOUT_SEC:
                status.data = 'tracking'
            else:
                status.data = 'lost'

        self._status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = VisionDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
