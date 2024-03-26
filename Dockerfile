FROM ubuntu:jammy

# Necessary environment variables, these enable the Dockerfile to be used in a non-interactive manner
ENV ROS_DIST iron
ENV TZ=Europe/Helsinki
ENV DEBIAN_FRONTEND=noninteractive
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Initial setup, update/upgrade and install basic tools
RUN \
    apt update && apt upgrade -y \
    && apt install -y software-properties-common \
    && apt update \
    && apt install -y \
	curl \
	vim \
	tldr \
	tmux \
	xclip \
	gh \
	wget \
	cmake \
	python3-pip \
    && apt autoremove -y

# pip setup & installing mkdocs for documentation
RUN \
    echo "export PATH=\"$HOME/.local/bin:$PATH\"" >> ~/.bashrc \
    && pip install mkdocs

# Locale setup
RUN \
	apt-get update && apt-get install locales \
	&& locale-gen en_US en_US.UTF-8 \
	&& update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
	&& export LANG=en_US.UTF-8

# Install ROS2
RUN \
    add-apt-repository universe \
    && apt update \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg\
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt update && apt install -y ros-dev-tools \
    && apt update && apt upgrade -y\
    && apt install -y ros-iron-desktop \
    && echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc


# Install ROS utilities and drivers
RUN \
    apt update \
    && apt install -y \
	ros-iron-xacro \
	ros-iron-joint-state-publisher-gui \
        ros-iron-spinnaker-camera-driver \
	ros-iron-diagnostic-updater \
	ros-iron-diagnostic-msgs

# Install CUDA toolkit
#RUN \
#    wget -nv https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-ubuntu2204.pin \
#    && mv cuda-ubuntu2204.pin /etc/apt/preferences.d/cuda-repository-pin-600 \
#    && wget -nv https://developer.download.nvidia.com/compute/cuda/12.3.2/local_installers/cuda-repo-ubuntu2204-12-3-local_12.3.2-545.23.08-1_amd64.deb \
#    && dpkg -i cuda-repo-ubuntu2204-12-3-local_12.3.2-545.23.08-1_amd64.deb \
#    && cp /var/cuda-repo-ubuntu2204-12-3-local/cuda-*-keyring.gpg /usr/share/keyrings/ \
#    && apt-get update \
#    && apt-get -y install cuda-toolkit-12-3 \
#    && apt-get install -y cuda-drivers

# Clone and initialize test bench workspace
RUN \
    git clone https://github.com/dedelstein/autonomous-lab.git \
    && cd autonomous-lab/test_bench_ws \
    && ./init_ws.sh
    
# Install Intel RealSense Software & ROS Drivers
RUN \
    mkdir -p /etc/apt/keyrings \
    && curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | tee /etc/apt/keyrings/librealsense.pgp > /dev/null \
    && echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | tee /etc/apt/sources.list.d/librealsense.list \
    && apt update && apt install -y  \
	librealsense2-dkms \
	librealsense2-utils \
	ros-iron-realsense2-*

WORKDIR autonomous-lab/test_bench_ws/
