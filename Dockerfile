# Use the official ROS 2 Jazzy Desktop image as the base (includes Gazebo & RViz)
FROM osrf/ros:jazzy-desktop

# Set non-interactive timezone to prevent apt-get from hanging
ENV DEBIAN_FRONTEND=noninteractive

# 1. Install System and Perception Dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    libopencv-dev \
    libpcl-dev \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-vision-opencv \
    ros-jazzy-pcl-conversions \
    ros-jazzy-behaviortree-cpp \
    ros-jazzy-message-filters \
    ros-jazzy-tf2-ros \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-twist-stamper \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-slam-toolbox \
    && rm -rf /var/lib/apt/lists/*

# 2. Install Python dependencies for inspector_llm (LLM bridge)
RUN python3 -m pip install --break-system-packages \
    google-genai \
    jsonschema \
    numpy \
    scipy \
    Pillow


# 3. Setup the Workspace
WORKDIR /ros2_ws
COPY ./src /ros2_ws/src

# 4. Resolve ROS dependencies
# Initialize rosdep if not already done by the base image
RUN rosdep init || true
RUN rosdep update && rosdep install -y \
    --from-paths src \
    --ignore-src \
    --rosdistro jazzy \
    --skip-keys "inspector_bot inspector_vision inspector_interfaces inspector_llm"

# 5. Build the Workspace (Ensuring the custom BT plugin and Lifecycle nodes compile)
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"

# 6. Setup the Entrypoint
COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

# Default command if none is provided
CMD ["bash"]