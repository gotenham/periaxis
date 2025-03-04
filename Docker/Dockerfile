# Dockerfile outlines the series of steps to build the Docker image, each step adds a layer to the generated image.
# Use the ROS 2 Jazzy base image as foundation
FROM ros:jazzy-ros-base

# Set the working directory inside the container
WORKDIR /ros2_ws

# Install dependencies and build tools
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*
#	ros-${ROS_DISTRO}-rviz2 \             # Optional: Visualization tool, include after python3-colcon-common-extensions above

# Install Eigen linear algebra library to Docker
RUN apt-get update && \
    apt-get install -y ros-jazzy-eigen3-* && \
    rm -rf /var/lib/apt/lists/*
	
# Source ROS 2 and build the workspace
RUN /bin/bash -c "source /opt/ros/jazzy/setup.sh && colcon build --symlink-install"

# Set the container to run bash on startup with a sourced project environment by default
CMD ["bash", "-c", "source /opt/ros/jazzy/setup.sh && source /ros2_ws/install/setup.bash && /bin/bash"]