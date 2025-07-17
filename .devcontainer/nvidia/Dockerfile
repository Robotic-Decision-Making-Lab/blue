FROM ghcr.io/robotic-decision-making-lab/blue:jazzy-desktop-nvidia

# Install ROS dependencies
# This is done in a previous stage, but we include it again here in case anyone wants to
# add new dependencies during development
ENV USERNAME=ubuntu
ENV USER_WORKSPACE=/home/$USERNAME/ws_blue
ENV USER_GID=1000
ENV USER_UID=1000
WORKDIR $USER_WORKSPACE

COPY --chown=$USER_UID:$USER_GID . src/blue

ENV ROS2_LATEST_ARCHIVE_KEYRING="/usr/share/keyrings/ros2-latest-archive-keyring.gpg"
RUN sudo rm -f ${ROS2_LATEST_ARCHIVE_KEYRING}
RUN sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o ${ROS2_LATEST_ARCHIVE_KEYRING}

RUN sudo apt-get -q update \
    && rosdep update \
    && rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} --skip-keys="gz-transport12 gz-sim7 gz-math7 gz-msgs9 gz-plugin2" \
    && sudo apt-get autoremove -y \
    && sudo apt-get clean -y \
    && sudo rm -rf /var/lib/apt/lists/*

# Install debugging/linting Python packages
RUN python3 -m pip install \
    pre-commit \
    mypy

# Disable the setuputils installation warning
# This prevents us from needing to pin the setuputils version (which doesn't always work)
ENV PYTHONWARNINGS="ignore"
