ARG ROS_DISTRO=rolling
FROM ros:$ROS_DISTRO-ros-base as ci

LABEL maintainer="Evan Palmer"
LABEL maintainer-email="evanp922@gmail.com"

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /root/ws_blue

COPY . src/blue

# Install apt packages
RUN apt-get -q update \
    && apt-get -q -y upgrade \
    && apt-get -q install --no-install-recommends -y \
    git \
    wget \
    curl \
    sudo \
    clang \
    clang-format-14 \
    clang-tidy \
    clang-tools \
    software-properties-common \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# Install MAVROS dependencies prior to installing ROS dependencies
RUN [ "/bin/bash" , "-c" , "\
    wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
    && chmod +x install_geographiclib_datasets.sh \
    && sudo ./install_geographiclib_datasets.sh" ]

# Install all ROS dependencies
RUN apt-get -q update \
    && apt-get -q -y upgrade \
    && rosdep update \
    && rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} --as-root=apt:false \
    && rm -rf src \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

FROM ci as source

ENV ROS_UNDERLAY /root/ws_blue/install
ENV DEBIAN_FRONTEND=noninteractive
WORKDIR $ROS_UNDERLAY/..

COPY . src/blue

RUN apt-get -q update \
    && apt-get -q -y upgrade \
    && rosdep update \
    && rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} --as-root=apt:false \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" \
    && colcon build \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    --ament-cmake-args -DCMAKE_BUILD_TYPE=Release \
    --event-handlers desktop_notification- status- \
    # Update /ros_entrypoint.sh to source the new workspace
    && sed -i "s#/opt/ros/\$ROS_DISTRO/setup.bash#$ROS_UNDERLAY/setup.sh#g" /ros_entrypoint.sh

FROM ci as develop

ENV ROS_UNDERLAY /root/ws_blue/install
ENV DEBIAN_FRONTEND=noninteractive
WORKDIR $ROS_UNDERLAY/..

COPY . src/blue

RUN apt-get -q update \
    && apt-get -q -y upgrade \
    && rosdep update \
    && rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} --as-root=apt:false \
    && rm -rf src \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# Install development tools
RUN apt-get -q update \
    && apt-get -q -y upgrade \
    && apt-get -q install --no-install-recommends -y \
    python3-dev \
    python3-pip \
    iputils-ping \
    net-tools \
    gdb \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# Install debugging/linting Python packages
RUN pip3 install \
    pre-commit \
    mypy \
    isort \
    flake8 \
    black \
    setuptools==58.2.0

# Configure a new non-root user
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update && apt-get upgrade -y \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/* \
    && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc \
    && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc

ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/$USERNAME/.bashrc
