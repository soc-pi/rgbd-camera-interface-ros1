# Use ROS Noetic base image
FROM osrf/ros:noetic-desktop-full

# Set shell to bash
SHELL ["/bin/bash", "-c"]

# Install basic development tools
RUN apt-get update && apt-get install -y \
    build-essential \
    git \
    python3-pip \
    python3-catkin-tools \
    vim \
    wget \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Create a new user 'ros' and add to sudo group
RUN useradd -m ros && \
    echo "ros:ros" | chpasswd && \
    usermod -aG sudo ros

# Set up ROS environment for both root and ros user
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source /opt/ros/noetic/setup.bash" >> /home/ros/.bashrc

# Create catkin workspace
WORKDIR /workspace
RUN mkdir -p /workspace/src && \
    chown -R ros:ros /workspace

# Initialize catkin workspace
RUN source /opt/ros/noetic/setup.bash && \
    catkin init && \
    catkin build

# Source the workspace in bashrc for both users
RUN echo "source /workspace/devel/setup.bash" >> ~/.bashrc && \
    echo "source /workspace/devel/setup.bash" >> /home/ros/.bashrc

# Switch to ros user
USER ros

# Set the default command
CMD ["bash"]