# AirMouse development image: Ubuntu 24.04 + ROS 2 Jazzy desktop-full + Gazebo Harmonic.
# Used by VS Code ("Reopen in Container") and by docker compose (see docker-compose.yml).
FROM osrf/ros:jazzy-desktop-full

ARG USERNAME=ubuntu

# ros_gz brings Gazebo Harmonic (the Gazebo release paired with Jazzy) and the ROS <-> Gazebo bridge.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ros-gz \
        ros-jazzy-rtabmap-ros \
        ros-jazzy-robot-localization \
        ros-jazzy-teleop-twist-keyboard \
        python3-numpy \
        python3-scipy \
        python3-pytest \
        python3-yaml \
        python3-colcon-common-extensions \
        sudo \
        bash-completion \
    && rm -rf /var/lib/apt/lists/*

# The Ubuntu 24.04 base already has user "ubuntu" (UID 1000), matching the usual host user,
# so files created in the mounted repo keep the host user's ownership.
RUN echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && (getent group render || groupadd --system render) \
    && usermod -aG video,render ${USERNAME}

# Every new shell gets ROS, and the workspace overlay once it has been built.
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /home/${USERNAME}/.bashrc \
    && echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> /home/${USERNAME}/.bashrc

USER ${USERNAME}
WORKDIR /ws
CMD ["bash"]
