FROM ghcr.io/robotic-decision-making-lab/blue:jazzy-desktop-nvidia


USER root

# Setup ROS2 repository and update package lists
RUN rm -f /etc/apt/sources.list.d/ros2*.list || true && \
    apt-key del F42ED6FBAB17C654 || true && \
    rm -f /usr/share/keyrings/ros*-archive-keyring.gpg || true && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update

# Install ROS dependencies
# This is done in a previous stage, but we include it again here in case anyone wants to
# add new dependencies during development
ENV USERNAME=ubuntu
ENV USER_WORKSPACE=/home/$USERNAME/ws_blue
WORKDIR $USER_WORKSPACE

COPY --chown=$USER_UID:$USER_GID . src/blue
RUN sudo apt-get -q update \
    && sudo apt-get -q -y upgrade \
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
