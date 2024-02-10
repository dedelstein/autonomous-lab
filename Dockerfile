FROM ubuntu:jammy

ENV ROS_PATH /root/dev/ros2_ws
ENV ROS_DIST iron

# Initial setup
RUN \
    apt update && apt upgrade -y \
    && apt install -y software-properties-common \
    && apt update \
    && apt install -y \
        curl \
        wget \
    	vim \
	    tldr \
	    tmux \
	    xclip \
	    gh \
        wget \
        cmake \
    # quick hack to run VLC as root
    && sed -i 's/geteuid/getppid/' /usr/bin/vlc \
    && apt autoremove

# Install ROS2
RUN \
    add-apt-repository universe \
    && apt update \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg\
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt update && apt install ros-dev-tools \\
    && apt update && apt upgrade \
    && apt install ros-iron-desktop \
    && echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc


# Install FLIR driver and other ROS drivers
RUN \
    apt update \
    && apt install -y \
        ros-iron-spinnaker-camera-driver

# Install CUDA toolkit
RUN \
    wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-ubuntu2204.pin \
    && mv cuda-ubuntu2204.pin /etc/apt/preferences.d/cuda-repository-pin-600 \
    && wget https://developer.download.nvidia.com/compute/cuda/12.3.2/local_installers/cuda-repo-ubuntu2204-12-3-local_12.3.2-545.23.08-1_amd64.deb \
    && dpkg -i cuda-repo-ubuntu2204-12-3-local_12.3.2-545.23.08-1_amd64.deb \
    && cp /var/cuda-repo-ubuntu2204-12-3-local/cuda-*-keyring.gpg /usr/share/keyrings/ \
    && apt-get update \
    && apt-get -y install cuda-toolkit-12-3 \
    && apt-get install -y cuda-drivers