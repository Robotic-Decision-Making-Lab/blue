#
# Dockerfile for *-robot development container
#
ARG BLUE_GITHUB_REPO=Robotic-Decision-Making-Lab/blue
ARG ROS_DISTRO=rolling
FROM ${BLUE_GITHUB_REPO}:${ROS_DISTRO}-robot

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
