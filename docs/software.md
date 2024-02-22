# Overview of Software Architecture

OS: Ubuntu 22.04 LTS  
ROS Distro: [Iron](https://docs.ros.org/en/iron/Releases/Release-Iron-Irwini.html)  
  
Drivers:  
 - [sick_scan_xd](https://github.com/SICKAG/sick_scan_xd/)  
 - [spinnaker camera driver](https://index.ros.org/p/spinnaker_camera_driver/)  

## Setup:
### Setting up Host Machine

Install Ubuntu 22.04, update, & install necessary software:  
 - [Docker](https://docs.docker.com/engine/install/ubuntu/)  
 - [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)  
 - [Rocker](https://github.com/osrf/rocker)  
 - [Git](https://github.com/git-guides/install-git)


```bash
# Install Docker:

# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null \
sudo apt-get update

# Install Nvidia container toolkit:
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit

# Install Rocker:

sudo apt-get install python3-rocker

# Install Git:

sudo apt-get install git-all
```

### Setting up the Test Bench
```bash
# Clone the remote git repository
git clone https://github.com/dedelstein/autonomous-lab.git && cd autonomous-lab

# Set up the ROS workspace
cd test_bench_ws && ./init_ws.sh
```

### To use a docker volume
```bash
# Build the docker image and create the volume (this takes time)
sudo docker image build --tag test-bench .
sudo docker volume create test-bench

# Mount the docker volume using rocker
sudo rocker --x11 --nvidia --privileged test-bench bash
```