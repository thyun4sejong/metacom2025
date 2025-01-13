# Stage 1: Base ROS 2 Humble setup
FROM osrf/ros:humble-desktop as ros_base

# Install additional ROS 2 packages required for navigation
RUN apt-get update && apt-get install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    python3-colcon-common-extensions \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Stage 2: Isaac Sim 4.2 setup
FROM nvcr.io/nvidia/isaac-sim:2023.1.0

# Copy ROS 2 installation from the ros_base stage
COPY --from=ros_base /opt/ros/humble /opt/ros/humble

# Set up environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    NVIDIA_DRIVER_CAPABILITIES=all \
    ROS_DISTRO=humble

ENV LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH


# Install required dependencies
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# Add ROS 2 GPG key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS 2 repository to sources.list.d
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list


# Install additional dependencies for Isaac Sim and ROS 2 integration
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-vision-msgs \
    libspdlog-dev

RUN pip3 install --no-cache-dir \
        rosdep \
        setuptools \
        vcstool

RUN apt-get clean && rm -rf /var/lib/apt/lists/*

# Set up Isaac Sim headless and ROS environment
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Copy navigation-related launch files and scripts
COPY ./workspace/ros2_navigation_launch /workspace/ros2_navigation_launch

# Set the working directory to Isaac Sim's user space
WORKDIR /isaac-sim

# Default command to launch Isaac Sim and ROS 2 Navigation
CMD ["/isaac-sim/python.sh", "-m", "omni.isaac.sim.headless.app", "--/app/renderer/headless=true", "--/workspace/ros2_navigation_launch/run_navigation_example.py"]
