cmake_minimum_required(VERSION 3.8)
project(inspector_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Suppress PCL FLANN_ROOT policy warning
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(inspector_interfaces REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(pluginlib REQUIRED)

add_executable(target_locator src/target_locator.cpp)
target_include_directories(target_locator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${OpenCV_INCLUDE_DIRS})
target_link_libraries(target_locator
  ${OpenCV_LIBRARIES}
)
target_compile_features(target_locator PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  target_locator
  "rclcpp"
  "sensor_msgs"
  "geometry_msgs"
  "cv_bridge"
  "pcl_conversions"
  "message_filters"
  "tf2_ros"
  "tf2_geometry_msgs"
  "rclcpp_lifecycle"
  "inspector_interfaces"
)

install(TARGETS target_locator
  DESTINATION lib/${PROJECT_NAME})

# Build the BT Plugin as a Shared Library
add_library(locate_target_bt_node SHARED src/locate_target_bt_node.cpp)
ament_target_dependencies(locate_target_bt_node
  "rclcpp"
  "rclcpp_action"
  "nav2_behavior_tree"
  "behaviortree_cpp"
  "inspector_interfaces"
  "pluginlib"
)

install(TARGETS
  locate_target_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

pluginlib_export_plugin_description_file(nav2_behavior_tree plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
ament_export_libraries(locate_target_bt_node)
ament_package()
