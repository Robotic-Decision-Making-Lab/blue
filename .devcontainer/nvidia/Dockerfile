FROM ghcr.io/robotic-decision-making-lab/blue:humble-desktop-nvidia

# Install ROS dependencies
# This is done in a previous stage, but we include it again here in case anyone wants to
# add new dependencies during development
ENV USERNAME=blue
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
COPY --chown=$USER_UID:$USER_GID requirements-dev.txt .
RUN python3 -m pip install -r requirements-dev.txt \
    && rm -rf requirements-dev.txt

# Disable the setuputils installation warning
# This prevents us from needing to pin the setuputils version (which doesn't always work)
ENV PYTHONWARNINGS="ignore"
