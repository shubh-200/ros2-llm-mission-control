#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <lifecycle_msgs/msg/state.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// Assuming you generated this custom interface
#include "inspector_interfaces/action/locate_target.hpp"

using rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using LocateTarget = inspector_interfaces::action::LocateTarget;
using GoalHandleLocateTarget = rclcpp_action::ServerGoalHandle<LocateTarget>;
using namespace std::placeholders;

class TargetLocatorNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    TargetLocatorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : rclcpp_lifecycle::LifecycleNode("target_locator", options)
    {}

    // =========================================================================
    // 1. LIFECYCLE STATE MACHINE
    // =========================================================================

    // Bootup: Allocate memory, setup the Action Server, but DO NOT connect to sensors.
    LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Configuring: Allocating Action Server and TF Broadcaster...");
        
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        action_server_ = rclcpp_action::create_server<LocateTarget>(
            this,
            "locate_target",
            std::bind(&TargetLocatorNode::handle_goal, this, _1, _2),
            std::bind(&TargetLocatorNode::handle_cancel, this, _1),
            std::bind(&TargetLocatorNode::handle_accepted, this, _1)
        );

        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    // Triggered by Nav2: Connect to Gazebo network and open the data floodgates.
    LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State & state)
    {
        RCLCPP_INFO(get_logger(), "Activating: Connecting to RGB-D sensor streams...");
        
        LifecycleNode::on_activate(state);

        // 1. Define the QoS profile natively
        rclcpp::QoS custom_qos = rclcpp::SensorDataQoS();
        
        // 2. Initialize the pointers EMPTY (This bypasses all std::make_unique template deduction errors)
        image_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::Image, rclcpp_lifecycle::LifecycleNode>>();
        pc_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::PointCloud2, rclcpp_lifecycle::LifecycleNode>>();

        // 3. Manually call subscribe, explicitly casting the string and extracting the base rmw_qos profile
        image_sub_->subscribe(this, std::string("/camera/image"), custom_qos.get_rmw_qos_profile());
        pc_sub_->subscribe(this, std::string("/camera/points"), custom_qos.get_rmw_qos_profile());

        // 4. Bind the synchronizer
        sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), *image_sub_, *pc_sub_);
        
        sync_->registerCallback(std::bind(&TargetLocatorNode::sensor_callback, this, _1, _2));

        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    // Triggered by Nav2: Task complete. Destroy the network connections immediately.
    LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state)
    {
        RCLCPP_INFO(get_logger(), "Deactivating: Severing sensor connections to save bandwidth...");
        
        LifecycleNode::on_deactivate(state);
        
        // Destroy pointers to drop the network subscriptions
        sync_.reset();
        image_sub_.reset();
        pc_sub_.reset();

        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
    {
        action_server_.reset();
        tf_broadcaster_.reset();
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state)
    {
        (void)state;
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

private:
    // =========================================================================
    // 2. ACTION SERVER LOGIC
    // =========================================================================
    
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const LocateTarget::Goal> goal)
    {
        (void)uuid;
        // Security Gate: Reject requests if the node is not mathematically Active
        if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
            RCLCPP_WARN(get_logger(), "Rejecting goal: Vision Node is currently INACTIVE.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        RCLCPP_INFO(get_logger(), "Received goal to locate target: %s", goal->target_id.c_str());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleLocateTarget> goal_handle)
    {
        (void)goal_handle;  
        RCLCPP_INFO(get_logger(), "Received request to cancel vision processing.");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleLocateTarget> goal_handle)
    {
        // Spawns a separate thread to process the vision pipeline so we don't block the node
        std::thread{std::bind(&TargetLocatorNode::execute_vision_pipeline, this, _1), goal_handle}.detach();
    }

    void execute_vision_pipeline(const std::shared_ptr<GoalHandleLocateTarget> goal_handle)
    {
        auto feedback = std::make_shared<LocateTarget::Feedback>();
        auto result = std::make_shared<LocateTarget::Result>();
        
        target_locked_ = false;

        RCLCPP_INFO(get_logger(), "Executing Vision Pipeline...");

        // Wait until the sensor_callback processes a frame and sets target_locked_ = true
        rclcpp::Rate loop_rate(10);
        while (rclcpp::ok() && !target_locked_) {
            if (goal_handle->is_canceling()) {
                result->success = false;
                goal_handle->canceled(result);
                return;
            }
            feedback->status = "Scanning optical feed...";
            goal_handle->publish_feedback(feedback);
            loop_rate.sleep();
        }

        // Once locked, succeed the action
        if (rclcpp::ok()) {
            result->success = true;
            result->final_pose = calculated_pose_;
            goal_handle->succeed(result);
            RCLCPP_INFO(get_logger(), "Target Locked. Action Succeed.");
        }
    }

    // =========================================================================
    // 3. SENSOR PROCESSING (OPENCV & PCL)
    // =========================================================================

    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::PointCloud2> SyncPolicy;

    void sensor_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr& img_msg,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pc_msg)
    {
        if (target_locked_) return; // Ignore frames if we aren't actively searching

        // ==========================================================
        // 1. cv_bridge: ROS Image -> OpenCV Matrix
        // ==========================================================
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // ==========================================================
        // 2. OpenCV: Grayscale & ArUco Detection
        // ==========================================================
        cv::Mat gray_img;
        cv::cvtColor(cv_ptr->image, gray_img, cv::COLOR_BGR2GRAY);
        
        // Modern OpenCV 4.x Syntax for AprilTag 36h11
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
        cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
        
        parameters->minDistanceToBorder = 0;  // Allow detection closer to edges
        parameters->minMarkerDistanceRate = 0.01;  // Lower threshold

        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
        std::vector<int> markerIds;
        cv::aruco::detectMarkers(gray_img, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

        // If no tag is seen, exit early and wait for the next frame
        if (markerIds.empty()) return; 

        RCLCPP_INFO(get_logger(), "AprilTag detected! Calculating 3D spatial geometry...");

        // ==========================================================
        // 3. Mathematical Mapping: 2D Center -> 3D Point Cloud
        // ==========================================================
        // Calculate the sub-pixel 2D center of the bounding box
        float u = (markerCorners[0][0].x + markerCorners[0][1].x + markerCorners[0][2].x + markerCorners[0][3].x) / 4.0;
        float v = (markerCorners[0][0].y + markerCorners[0][1].y + markerCorners[0][2].y + markerCorners[0][3].y) / 4.0;

        // Convert ROS PointCloud2 to organized PCL object
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        pcl::fromROSMsg(*pc_msg, cloud);

        int u_int = static_cast<int>(u);
        int v_int = static_cast<int>(v);

        // Security check: Ensure coordinates are inside the camera frame
        if (u_int < 0 || u_int >= static_cast<int>(cloud.width) || 
            v_int < 0 || v_int >= static_cast<int>(cloud.height)) return;

        // Extract the strict physical depth metric
        pcl::PointXYZRGB target_point = cloud.at(u_int, v_int);

        // Security check: Filter out NaN anomalies common in raw LiDAR/Depth streams
        if (std::isnan(target_point.x) || std::isnan(target_point.y) || std::isnan(target_point.z)) {
            RCLCPP_WARN(get_logger(), "Target detected, but depth data is NaN. Waiting for cleaner frame...");
            return;
        }

        // ==========================================================
        // 4. Populate the Output Contract
        // ==========================================================
        calculated_pose_.header = pc_msg->header; // Inherit the strict timestamp and camera frame ID
        calculated_pose_.pose.position.x = target_point.x;
        calculated_pose_.pose.position.y = target_point.y;
        calculated_pose_.pose.position.z = target_point.z;
        calculated_pose_.pose.orientation.w = 1.0; // Identity orientation 

        // ==========================================================
        // 5. Broadcast to the TF Tree
        // ==========================================================
        geometry_msgs::msg::TransformStamped t;

        // Sync the time and frame exactly with the camera's optical frame
        t.header.stamp = pc_msg->header.stamp;
        t.header.frame_id = pc_msg->header.frame_id; 
        t.child_frame_id = "cargo_target";

        // Map the PCL coordinates
        t.transform.translation.x = target_point.x;
        t.transform.translation.y = target_point.y;
        t.transform.translation.z = target_point.z;

        // Identity quaternion (no rotation applied yet)
        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = 0.0;
        t.transform.rotation.w = 1.0;

        // Send it to the ROS network!
        tf_broadcaster_->sendTransform(t);

        // Trigger the success flag to close the Action Server
        target_locked_ = true;
    }

    // Class Members
    rclcpp_action::Server<LocateTarget>::SharedPtr action_server_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::Image, rclcpp_lifecycle::LifecycleNode>> image_sub_;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2, rclcpp_lifecycle::LifecycleNode>> pc_sub_;
    std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    bool target_locked_ = false;
    geometry_msgs::msg::PoseStamped calculated_pose_;
};

// Must use standard main block for Lifecycle Nodes
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<TargetLocatorNode>();
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}