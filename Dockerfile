FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip python3-colcon-common-extensions python3-colcon-ros \
        ros-jazzy-joy joystick i2c-tools \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --break-system-packages transforms3d pigpio

RUN echo "source /opt/ros/jazzy/setup.bash && [ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash" >> /root/.bashrc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /ros2_ws
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
