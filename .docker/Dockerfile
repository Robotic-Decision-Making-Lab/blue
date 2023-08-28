##############################################
# ci: Install core dependencies needed for CI
##############################################
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
    python3-pip \
    python3-dev \
    software-properties-common \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# Install the Python requirements that aren't available as rosdeps
RUN python3 -m pip install -r $(pwd)/src/blue/requirements-build.txt

# Install gstreamer
RUN apt-get -q update \
    && apt-get -q -y upgrade \
    && apt-get -q install --no-install-recommends -y \
    python3-gi \
    gstreamer1.0-tools \
    gir1.2-gstreamer-1.0 \
    gir1.2-gst-plugins-base-1.0 \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# Install all ROS dependencies
RUN apt-get -q update \
    && apt-get -q -y upgrade \
    && rosdep update \
    && rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} --as-root=apt:false \
    && rm -rf src \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# Configure a new non-root user
ARG USERNAME=blue
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc

# Switch to the non-root user to install MAVROS dependencies
USER $USERNAME
ENV USER=$USERNAME

# Install MAVROS dependencies
WORKDIR /home/$USERNAME
RUN wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh \
    && chmod +x install_geographiclib_datasets.sh \
    && sudo ./install_geographiclib_datasets.sh

# Switch back to root user
USER root
ENV USER=root

#################################################################
# sim: install Gazebo and the tools needed for Gazebo simulation
#################################################################
FROM ci as sim

# Install Gazebo Garden: https://gazebosim.org/docs/garden/install_ubuntu
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update \
    && apt-get -y --quiet --no-install-recommends install \
    gz-garden \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# Install ArduPilot and ardupilot_gazebo dependencies
RUN apt-get -q update \
    && apt-get -q -y upgrade \
    && apt-get -q install --no-install-recommends -y \
    python3-wxgtk4.0 \
    rapidjson-dev \
    xterm \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# Switch to the non-root user to install ArduSub
USER $USERNAME
ENV USER=$USERNAME

# Clone ArduSub
# ArduSub is installed for simulation purposes ONLY
# When deployed onto hardware, the native installation of ArduSub
# (on the FCU) will be used.
WORKDIR /home/$USERNAME
RUN git clone https://github.com/ArduPilot/ardupilot.git --recurse-submodules

# Install ArduSub dependencies
WORKDIR /home/$USERNAME/ardupilot
ENV SKIP_AP_EXT_ENV=1 SKIP_AP_GRAPHIC_ENV=1 SKIP_AP_COV_ENV=1 SKIP_AP_GIT_CHECK=1
RUN Tools/environment_install/install-prereqs-ubuntu.sh -y

# Build ArduSub
WORKDIR /home/$USERNAME/ardupilot
RUN modules/waf/waf-light configure --board sitl \
    && modules/waf/waf-light build --target bin/ardusub

# Clone ardupilot_gazebo code
WORKDIR /home/$USERNAME
RUN git clone https://github.com/ArduPilot/ardupilot_gazebo.git

# Install ardupilot_gazebo plugin
RUN [ "/bin/bash" , "-c" , " \
    cd ardupilot_gazebo \
    && mkdir build \
    && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    && make -j4" ]

# Switch back to the root user
USER root
ENV USER=root

# Extend the ros_entrypoint to source the simulation environment
COPY .docker/entrypoints/sim.sh /
RUN sed -i '/source "\/opt\/ros\/$ROS_DISTRO\/setup\.bash" --/a source /sim.sh' /ros_entrypoint.sh

##################################################
# deps: Install all external project dependencies
##################################################
FROM sim as deps

ENV GZ_VERSION=garden
ENV USER_WORKSPACE=/home/$USERNAME/ws_blue/install
ENV DEBIAN_FRONTEND=noninteractive
WORKDIR $USER_WORKSPACE/..

COPY blue.repos src/

RUN apt-get -q update \
    && apt-get -q -y upgrade \
    && vcs import src < src/blue.repos \
    && rosdep update \
    && rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} --as-root=apt:false --skip-keys="gz-transport12 gz-sim7 gz-math7 gz-msgs9 gz-plugin2" \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" \
    && colcon build \
    # Update /ros_entrypoint.sh to source the workspace
    && sed -i "s#/opt/ros/\$ROS_DISTRO/setup.bash#$USER_WORKSPACE/setup.sh#g" /ros_entrypoint.sh \
    && echo "source ${USER_WORKSPACE}/setup.sh" >> /home/$USERNAME/.bashrc

####################################################
# develop: Setup the image for development purposes
####################################################
FROM deps as develop

ENV GZ_VERSION=garden
ENV ROS_UNDERLAY /root/ws_blue/install
ENV DEBIAN_FRONTEND=noninteractive
WORKDIR $ROS_UNDERLAY/..

COPY . src/blue

# Install debugging/linting Python packages
RUN python3 -m pip install -r $(pwd)/src/blue/requirements-dev.txt

RUN apt-get -q update \
    && apt-get -q -y upgrade \
    && vcs import src < src/blue/blue.repos \
    && rosdep update \
    && rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} --as-root=apt:false --skip-keys="gz-transport12 gz-sim7 gz-math7 gz-msgs9" \
    && rm -rf src \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# Install development tools
RUN apt-get -q update \
    && apt-get -q -y upgrade \
    && apt-get -q install --no-install-recommends -y \
    iputils-ping \
    net-tools \
    gdb \
    nano \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

COPY .docker/entrypoints/dev.sh /

# WARNING: This is a temporary solution for disabling the setuputils installation warning
ENV PYTHONWARNINGS="ignore"

ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/$USERNAME/.bashrc \
    && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc \
    # Expose the environment variables to the non-root user
    && echo "if [ -f /sim.sh ]; then source /sim.sh; fi" >> /home/$USERNAME/.bashrc \
    && echo "if [ -f /dev.sh ]; then source /dev.sh; fi" >> /home/$USERNAME/.bashrc


######################################################################################
# develop-nvidia: Setup the image for development purposes with NVIDIA driver support
######################################################################################
FROM deps as develop-nvidia

# Install NVIDIA software
RUN apt-get update \
    && apt-get install -y -qq --no-install-recommends \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6

# Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute
ENV QT_X11_NO_MITSHM 1

ENV GZ_VERSION=garden
ENV ROS_UNDERLAY /root/ws_blue/install
ENV DEBIAN_FRONTEND=noninteractive
WORKDIR $ROS_UNDERLAY/..

COPY . src/blue

# Install debugging/linting Python packages
RUN python3 -m pip install -r $(pwd)/src/blue/requirements-dev.txt

RUN apt-get -q update \
    && apt-get -q -y upgrade \
    && vcs import src < src/blue/blue.repos \
    && rosdep update \
    && rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} --as-root=apt:false --skip-keys="gz-transport12 gz-sim7 gz-math7 gz-msgs9 gz-plugin2" \
    && rm -rf src \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# Install development tools
RUN apt-get -q update \
    && apt-get -q -y upgrade \
    && apt-get -q install --no-install-recommends -y \
    iputils-ping \
    net-tools \
    gdb \
    nano \
    htop \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

COPY .docker/entrypoints/dev.sh /

# WARNING: This is a temporary solution for disabling the setuputils installation warning
ENV PYTHONWARNINGS="ignore"

ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/$USERNAME/.bashrc \
    && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc \
    # Expose the environment variables to the non-root user
    && echo "if [ -f /sim.sh ]; then source /sim.sh; fi" >> /home/$USERNAME/.bashrc \
    && echo "if [ -f /dev.sh ]; then source /dev.sh; fi" >> /home/$USERNAME/.bashrc
