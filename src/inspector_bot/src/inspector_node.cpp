#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
// #include <logging.hpp>

// 1. Defining the Node Class Interface
class InspectorNode : public rclcpp::Node 
{
public:
  InspectorNode() : Node("inspector_node") 
  {
    RCLCPP_INFO(this->get_logger(), "Inspector Bot Node Initialized successfully.");
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

// 2. The Execution Entry Point (The Linker looks for exactly this!)
int main(int argc, char * argv[]) 
{
  // rclcpp::init maps directly to rclpy.init(args=args) in Python.
  // It initializes the underlying DDS middleware layers.
  rclcpp::init(argc, argv);
  
  // std::make_shared allocates memory for your class instance dynamically.
  // rclcpp::spin maps directly to rclpy.spin(node) in Python.
  // It keeps the node alive, listening for timer events or incoming topic messages.
  rclcpp::spin(std::make_shared<InspectorNode>());
  
  // Clean up resources once shutdown signal (Ctrl+C) is received
  rclcpp::shutdown();
  return 0;
}