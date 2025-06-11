# FROM ghcr.io/jbvakshaya/blue:iron-nvidia-ompl
FROM ghcr.io/jbvakshaya/blue:iron-nvidia-multi-robots-recent
# With above image we probably don't need to install any other dependencies

# FROM ghcr.io/robotic-decision-making-lab/blue:iron-desktop-nvidia

# Install ROS dependencies
# This is done in a previous stage, but we include it again here in case anyone wants to
# add new dependencies during development
ENV USERNAME=ubuntu
ENV USER_WORKSPACE=/home/$USERNAME/ws_blue
WORKDIR $USER_WORKSPACE

COPY --chown=$USER_UID:$USER_GID . src/blue
# RUN sudo apt-get -q update \
#     && sudo apt-get -q -y upgrade \
#     && rosdep update \
#     && rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} --skip-keys="gz-transport12 gz-sim7 gz-math7 gz-msgs9 gz-plugin2" \
#     && sudo apt-get autoremove -y \
#     && sudo apt-get clean -y \
#     && sudo rm -rf /var/lib/apt/lists/*

# # Install OMPL with ROS2
# RUN curl http://repo.ros2.org/repos.key | sudo apt-key add -\
#     && sudo apt-get update \
#     && sudo apt-get install -y nano \
#     && sudo apt-get install -y ros-iron-ompl

# RUN cd ${USER_WORKSPACE}/src/learn_ros2 \
#     && echo "Check me" \
#     && echo ls -d -- */

# Install pinocchio
RUN sudo apt update && sudo apt install -y ros-${ROS_DISTRO}-pinocchio

# Update rqt to find plugins work with ROS2
RUN  sudo apt update \
    && sudo apt install -y ros-${ROS_DISTRO}-rqt*

# tf2 depedencies
# RUN sudo apt-get install ros-${ROS_DISTRO}-rviz2 ros-${ROS_DISTRO}-turtle-tf2-py ros-${ROS_DISTRO}-tf2-ros ros-${ROS_DISTRO}-tf2-tools ros-${ROS_DISTRO}-turtlesim

# Install the learn_ros2 package
# RUN cd ${USER_WORKSPACE} \
#     && echo "Check me" \
#     && pwd \
#     && colcon build --packages-select learn_ros2 \
#     && . install/setup.bash

RUN echo 'nvidia' > ${USER_WORKSPACE}/nvidia.txt
# Install debugging/linting Python packages
RUN python3 -m pip install \
    pre-commit \
    mypy

# Disable the setuputils installation warning
# This prevents us from needing to pin the setuputils version (which doesn't always work)
ENV PYTHONWARNINGS="ignore"
