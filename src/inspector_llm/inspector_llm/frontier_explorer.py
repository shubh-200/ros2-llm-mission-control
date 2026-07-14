"""
Frontier-based exploration for SLAM mode.

Subscribes to the live /map topic (OccupancyGrid published by SLAM Toolbox),
detects frontier cells (free cells adjacent to unknown space), clusters them,
and returns centroid waypoints for the robot to explore.
"""

import numpy as np
from scipy import ndimage
from nav_msgs.msg import OccupancyGrid
import rclpy


# --- OccupancyGrid cell values ---
CELL_UNKNOWN = -1
CELL_FREE = 0
# Anything > 0 is occupied (higher = more certain)
CELL_OCCUPIED_THRESH = 50

# --- Frontier filtering ---
MIN_FRONTIER_SIZE = 15         # Ignore tiny frontier clusters (noise)
MIN_DISTANCE_FROM_ROBOT = 2.0  # metres — don't target frontiers right under the robot
MAX_DISTANCE_FROM_ROBOT = 20.0 # metres — don't target frontiers too far away


class FrontierExplorer:
    """Detects frontiers in a live OccupancyGrid and returns exploration waypoints."""

    def __init__(self, node):
        self._node = node
        self._map_msg: OccupancyGrid | None = None

        self._sub = node.create_subscription(
            OccupancyGrid,
            '/map',
            self._map_callback,
            10
        )

    def _map_callback(self, msg: OccupancyGrid):
        self._map_msg = msg

    def wait_for_map(self, node, timeout_sec=30.0):
        """Block until the first /map message arrives."""
        import time
        start = time.time()
        while self._map_msg is None:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.time() - start > timeout_sec:
                raise RuntimeError('Timed out waiting for /map from SLAM Toolbox')
        node.get_logger().info('Map received from SLAM Toolbox.')

    def get_frontiers(self, robot_x: float = 0.0, robot_y: float = 0.0,
                      max_count: int = 5) -> list[tuple[float, float]]:
        """
        Find frontier waypoints in the current map.

        Returns a list of (x, y) world coordinates, sorted by exploration
        value (largest frontiers preferred, with distance as tiebreaker),
        capped at max_count.
        """
        if self._map_msg is None:
            return []

        # Spin once to get latest map
        rclpy.spin_once(self._node, timeout_sec=0.05)

        msg = self._map_msg
        info = msg.info
        width = info.width
        height = info.height
        resolution = info.resolution
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y

        # --- 1. Reshape to 2D grid ---
        grid = np.array(msg.data, dtype=np.int8).reshape((height, width))

        # --- 2. Create masks ---
        # Include cells with low cost (0-10) as navigable, not just exactly 0
        free_mask = (grid >= CELL_FREE) & (grid <= 10)
        unknown_mask = (grid == CELL_UNKNOWN)

        # --- 3. Find frontier cells ---
        # A frontier cell is a FREE cell that has at least one UNKNOWN neighbor.
        # Dilate the unknown mask by 1 pixel (4-connectivity) and AND with free.
        struct = ndimage.generate_binary_structure(2, 1)  # 4-connected
        unknown_dilated = ndimage.binary_dilation(unknown_mask, structure=struct)
        frontier_mask = free_mask & unknown_dilated

        # --- 4. Cluster frontiers ---
        labeled, num_features = ndimage.label(frontier_mask, structure=struct)

        if num_features == 0:
            self._node.get_logger().info('No frontiers found — map may be fully explored.')
            return []

        # --- 5. Compute centroids and filter ---
        waypoints = []

        for label_id in range(1, num_features + 1):
            cluster_mask = (labeled == label_id)
            cluster_size = np.sum(cluster_mask)

            # Skip tiny clusters (noise)
            if cluster_size < MIN_FRONTIER_SIZE:
                continue

            # Centroid in grid coordinates (row, col)
            rows, cols = np.where(cluster_mask)
            centroid_row = np.mean(rows)
            centroid_col = np.mean(cols)

            # Convert grid → world coordinates
            world_x = origin_x + centroid_col * resolution
            world_y = origin_y + centroid_row * resolution

            # Distance from robot
            dist = np.sqrt((world_x - robot_x) ** 2 + (world_y - robot_y) ** 2)

            # Filter by distance
            if dist < MIN_DISTANCE_FROM_ROBOT:
                continue
            if dist > MAX_DISTANCE_FROM_ROBOT:
                continue

            # Check that the centroid itself is not occupied
            c_row = int(centroid_row)
            c_col = int(centroid_col)
            if 0 <= c_row < height and 0 <= c_col < width:
                if grid[c_row, c_col] > CELL_OCCUPIED_THRESH:
                    continue

            waypoints.append((world_x, world_y, dist, cluster_size))

        # --- 6. Sort by exploration value ---
        # Primary: largest frontier (most unexplored area). Tiebreaker: closest.
        # This pushes the robot toward big unexplored zones instead of nibbling edges.
        waypoints.sort(key=lambda w: (-w[3], w[2]))

        # Return top N as (x, y) tuples
        result = [(w[0], w[1]) for w in waypoints[:max_count]]

        self._node.get_logger().info(
            f'Found {len(waypoints)} frontiers, returning top {len(result)}: '
            f'{[(f"{x:.2f}", f"{y:.2f}") for x, y in result]}'
        )

        return result
