FROM ros:jazzy-ros-base
RUN apt-get update && apt-get install -y \
    python3-pip python3-colcon-common-extensions python3-colcon-ros \
    ros-jazzy-joy joystick i2c-tools && rm -rf /var/lib/apt/lists/*
RUN pip3 install transforms3d pigpio --break-system-packages
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc && \
    echo '[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash' >> /root/.bashrc
WORKDIR /ros2_ws
