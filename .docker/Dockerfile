ARG ROS_DISTRO=rolling
FROM ros:$ROS_DISTRO-ros-base AS base

# Create the non-root user early because both "mavros" and "robot"
# stages depend its existence
RUN apt-get -q update \
    && apt-get -q -y upgrade \
    && apt-get -q install --no-install-recommends -y \
    git \
    gosu \
    sudo \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# Ubuntu 24.04 "Noble", which is used as the base image for
# jazzy and rolling images, now includes a user "ubuntu" at UID 1000
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && usermod -a -G dialout $USERNAME \
    && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc

ENV DEBIAN_FRONTEND=noninteractive
ENV USER=$USERNAME

# This needs to be defined in a shared stage so it's available in both "mavros" and "robot"
ENV MAVROS_WORKSPACE=/home/$USERNAME/ws_mavros

#== Build Mavros/Mavlink from source in its own workspace.
#   Extend that workspace into the "blue" workspace
#
FROM base AS mavros

ENV DEBIAN_FRONTEND=noninteractive
USER $USERNAME

WORKDIR $MAVROS_WORKSPACE/src

# As a reminder to self.   git clone by itself will not break cache
# when the repo changes.   To rebuild mavros from the latest, you
# must "docker build --no-cache" to force a rebuild
ARG MAVROS_RELEASE=ros2
ARG MAVLINK_RELEASE=release/rolling/mavlink
RUN git clone --depth 1 -b ${MAVROS_RELEASE} https://github.com/mavlink/mavros.git
RUN git clone --depth 1 --recursive -b ${MAVLINK_RELEASE}  https://github.com/ros2-gbp/mavlink-gbp-release.git mavlink
# Fix two issues currently in mavros/ros2
# - mavgen uses future.standard_library for backwards compatibility with Python2;
#   However, this caused issues with Python 3.12 installed in "noble".
#   Comment those lines out in mavlink.
#
# - Fix linkage for yaml-cpp in mavros_extra_plugins
RUN sed -i -e 's/^from future import standard_library/#from future import standard_library/' \
    -e 's/standard_library.install_aliases()/#standard_library.install_aliases()/' \
    mavlink/pymavlink/generator/mavgen.py && \
    sed -i -e 's/^# find_package(yaml_cpp REQUIRED)/find_package(yaml-cpp REQUIRED)/' \
    -e '/^ament_target_dependencies(mavros_extras_plugins$/i target_link_libraries(mavros_extras_plugins yaml-cpp::yaml-cpp)' \
    -e '/^ament_target_dependencies(mavros_extras$/i target_link_libraries(mavros_extras yaml-cpp::yaml-cpp)' \
    mavros/mavros_extras/CMakeLists.txt

WORKDIR $MAVROS_WORKSPACE
USER root
RUN apt-get -q update \
    && apt-get -q -y upgrade \
    && gosu $USERNAME rosdep update \
    && gosu $USERNAME rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*
USER $USERNAME

# Build Mavros workspace
RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" \
    && colcon build \
    && rm -rf $MAVROS_WORKSPACE/build

# == "Standard" Blue Docker image proceeds from here
#
FROM base AS ci

ENV DEBIAN_FRONTEND=noninteractive
USER root

WORKDIR /root/ws_blue
COPY . src/blue

# Install apt packages needed for CI
#
# Explicitly uninstall and block mavros and mavlink packages from APT
# this will trigger an error if rosdep attempts to install mavros
# from APT rather than using the version in $MAVROS_WORKSPACE
RUN apt-get -q update \
    && apt-get -q -y upgrade \
    && apt-get remove -y "*mavros*" "*mavlink*" \
    && apt-mark hold "*mavros*" "*mavlink*" \
    && apt-get -q install --no-install-recommends -y \
    clang \
    clang-format-14 \
    clang-tidy \
    clang-tools \
    python3-pip \
    python3-dev \
    python3-venv \
    lsb-release \
    wget \
    gnupg \
    software-properties-common \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# As it's a dependency for blue, copy Mavros from its build stage
COPY --from=mavros --chown=$USERNAME:$USERNAME $MAVROS_WORKSPACE $MAVROS_WORKSPACE

# Install all ROS dependencies for _just_ blue (and mavros)
# (we have not imported the other repos from blue.repos files)
RUN apt-get -q update \
    && apt-get -q -y upgrade \
    && rosdep update \
    && . "${MAVROS_WORKSPACE}/install/setup.sh" \
    && rosdep install -y --from-paths ${MAVROS_WORKSPACE}/src --ignore-src --rosdistro ${ROS_DISTRO} \
    && rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} \
    && rm -rf src \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# This stage includes
#   - Switching to the non-root user
#   - Copying 'blue' source from this repo into the non-root user's workspace
#   - Installing blue deps using pip, apt and rosdep
#   - Installs the remaining blue dependencies from blue_robot.repos
#   - Installs deps from rosdep for all src dependencies
#   - colcon build
#
FROM ci AS robot

# Switch to the non-root user for the rest of the installation
USER $USERNAME

# Python in Ubuntu is now marked as a "Externally managed environment",
# Per best practice, create a venv for local python packages
#
# These two ENVs effectively "activate" the venv for subsequent calls to
# python/pip in the Dockerfile
WORKDIR /home/$USERNAME
ENV VIRTUAL_ENV=/home/$USERNAME/.venv/blue
RUN python3 -m venv --system-site-packages --symlinks $VIRTUAL_ENV
ENV PATH="$VIRTUAL_ENV/bin:$PATH"

# Install MAVROS dependencies
WORKDIR /home/$USERNAME
RUN wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh \
    && chmod +x install_geographiclib_datasets.sh \
    && sudo ./install_geographiclib_datasets.sh

ENV USER_WORKSPACE=/home/$USERNAME/ws_blue
WORKDIR $USER_WORKSPACE
COPY --chown=$USER_UID:$USER_GID . src/blue

# Install the Python requirements that aren't available as rosdeps
RUN python3 -m pip install -r $(pwd)/src/blue/requirements-build.txt

# Install gstreamer
RUN sudo apt-get -q update \
    && sudo apt-get -q -y upgrade \
    && sudo apt-get -q install --no-install-recommends -y \
    python3-gi \
    gstreamer1.0-tools \
    gir1.2-gstreamer-1.0 \
    gir1.2-gst-plugins-base-1.0 \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    libgstreamer1.0-dev \
    gstreamer1.0-gl \
    libgstreamer-plugins-base1.0-dev \
    && sudo apt-get autoremove -y \
    && sudo apt-get clean -y \
    && sudo rm -rf /var/lib/apt/lists/*

WORKDIR $USER_WORKSPACE
RUN sudo apt-get -q update \
    && sudo apt-get -q -y upgrade \
    && vcs import src < src/blue/blue.repos \
    && rosdep update \
    && . "${MAVROS_WORKSPACE}/install/setup.sh" \
    && rosdep install -y --from-paths ${MAVROS_WORKSPACE}/src --ignore-src --rosdistro ${ROS_DISTRO} \
    && rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} \
    && sudo apt-get autoremove -y \
    && sudo apt-get clean -y \
    && sudo rm -rf /var/lib/apt/lists/*

# Actually build workspace
RUN . "${MAVROS_WORKSPACE}/install/setup.sh" \
    && colcon build

RUN echo "source ${USER_WORKSPACE}/install/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "source $VIRTUAL_ENV/bin/activate" >> /home/$USERNAME/.bashrc \
    && echo "\n# Ensure colcon is run in the venv\nalias colcon='python3 -m colcon'" >> /home/$USERNAME/.bashrc

FROM robot AS desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV GZ_VERSION=harmonic

# Install Gazebo Harmonic: https://gazebosim.org/docs/harmonic/install_ubuntu
# Per DL3004, use "USER root" rather than "sudo"
#    https://github.com/hadolint/hadolint/wiki/DL3004
USER root
# Install custom rosdep list
ADD --chown=root:root --chmod=0644 https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list /etc/ros/rosdep/sources.list.d/00-gazebo.list
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" |  tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get -q update \
    && apt-get -y --quiet --no-install-recommends install \
    cppzmq-dev \
    gz-${GZ_VERSION} \
    python3-pexpect \
    python3-wxgtk4.0 \
    python3-future \
    rapidjson-dev \
    xterm \
    rapidjson-dev \
    libopencv-dev \
    cppzmq-dev \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*
USER $USERNAME

# Clone ArduSub
# ArduSub is installed for simulation purposes ONLY
# When deployed onto hardware, the native installation of ArduSub
# (on the FCU) will be used.
WORKDIR /home/$USERNAME
# Really should do version pinning but Sub-4.5 is waaaay behind master
# (e.g. it doesn't know about "noble" yet)
ARG ARDUPILOT_RELEASE=master
RUN git clone -b ${ARDUPILOT_RELEASE} https://github.com/ArduPilot/ardupilot.git --recurse-submodules

# Install ArduSub dependencies
WORKDIR /home/$USERNAME/ardupilot
ENV SKIP_AP_EXT_ENV=1 SKIP_AP_GRAPHIC_ENV=1 SKIP_AP_COV_ENV=1 SKIP_AP_GIT_CHECK=1
# Do not install the STM development tools
ENV DO_AP_STM_ENV=0
# Do not activate the Ardupilot venv by default
ENV DO_PYTHON_VENV_ENV=0
RUN echo "\n# Below from ardupilot script \"install-prereqs-ubuntu.sh\"\n" >> /home/$USERNAME/.bashrc && \
    Tools/environment_install/install-prereqs-ubuntu.sh -y

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

# Install ros_gz and other project dependencies
WORKDIR $USER_WORKSPACE
RUN sudo apt-get -q update \
    && sudo apt-get -q -y upgrade \
    && vcs import src < src/blue/sim.repos \
    && rosdep update \
    && . "${MAVROS_WORKSPACE}/install/setup.sh" \
    && rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} \
    && sudo apt-get autoremove -y \
    && sudo apt-get clean -y \
    && sudo rm -rf /var/lib/apt/lists/*

# For users that build this on a laptop or system with limited RAM,
# Modify the 'colcon build' line to be 'MAKEFLAGS="-j1 -l1" colcon build'
# This will limit the amount of RAM that colcon is allowed to use
RUN . "${MAVROS_WORKSPACE}/install/setup.sh" \
    && colcon build

# Setup the simulation environment variables
RUN  <<EOT cat >> /home/$USERNAME/.bashrc

# Add results of ArduSub build
export PATH=\$HOME/ardupilot/build/sitl/bin:\$PATH

# Optional: add autotest to the PATH, helpful for running sim_vehicle.py
export PATH=\$HOME/ardupilot/Tools/autotest:\$PATH

# Add ardupilot_gazebo plugin
export GZ_SIM_SYSTEM_PLUGIN_PATH=\$HOME/ardupilot_gazebo/build:\$GZ_SIM_SYSTEM_PLUGIN_PATH

# Add ardupilot_gazebo models and worlds
export GZ_SIM_RESOURCE_PATH=\$HOME/ardupilot_gazebo/models:\$HOME/ardupilot_gazebo/worlds:\$GZ_SIM_RESOURCE_PATH

# Add blue models and worlds
BLUE_DESCRIPTION=\$USER_WORKSPACE/src/blue/blue_description
export GZ_SIM_RESOURCE_PATH=\$BLUE_DESCRIPTION/gazebo/models:\$BLUE_DESCRIPTION/gazebo/worlds:\$GZ_SIM_RESOURCE_PATH

# Add blue meshes
export GZ_SIM_RESOURCE_PATH=\$BLUE_DESCRIPTION/meshes:\$GZ_SIM_RESOURCE_PATH
EOT

FROM desktop AS desktop-nvidia

# Install NVIDIA software
RUN sudo apt-get update \
    && sudo apt-get -q -y upgrade \
    && sudo apt-get install -y -qq --no-install-recommends \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 \
    && sudo apt-get autoremove -y \
    && sudo apt-get clean -y \
    && sudo rm -rf /var/lib/apt/lists/*

# Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
ENV QT_X11_NO_MITSHM=1
