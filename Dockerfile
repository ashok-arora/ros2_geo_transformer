# Start from official ROS 2 Humble image
FROM ros:humble

# Install GeographicLib and other common dependencies
RUN apt-get update && apt-get install -y \
    geographiclib-tools \
    libgeographic-dev \
    build-essential \
    python3-colcon-common-extensions \
    git \
    && rm -rf /var/lib/apt/lists/*

# Setup colcon workspace
WORKDIR /ros2_ws
RUN mkdir src

# Copy your ROS 2 packages (or clone repos)
COPY ./src ./src

# Build workspace
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

# Source entrypoint
COPY ./ros_entrypoint.sh /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

