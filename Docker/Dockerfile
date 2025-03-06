# Dockerfile outlines the series of steps to build the Docker image, each step adds a layer to the generated image.
# Use the official ROS 2 Jazzy Jalisco base image as foundation
FROM ros:jazzy-ros-base

# Set the working directory inside the container
WORKDIR /ros2_ws

# Install dependencies and development tools:
    # Update to latest package list
    # Install: Colcon build tools for ROS 2
    # Install: Eigen3 library for linear algebra operations
    # Clean up to reduce image size
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# Ensure the ROS 2 environment is sourced every time a shell starts
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc

# Set up entrypoint to run an interactive bash shell by default
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && exec bash"]
#ENTRYPOINT ["/bin/bash"]