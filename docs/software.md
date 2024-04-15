# Overview of Software Architecture

OS: Ubuntu 22.04 LTS  
ROS Distro: [Iron](https://docs.ros.org/en/iron/Releases/Release-Iron-Irwini.html)  
  
ROS2 Drivers:  
 - [sick_scan_xd](https://github.com/SICKAG/sick_scan_xd/)  
 - [spinnaker camera driver](https://index.ros.org/p/spinnaker_camera_driver/)  
 - [BNO055 driver](https://github.com/flynneva/bno055)  
 - [Marvelmind RTLS driver](https://github.com/MarvelmindRobotics/marvelmind_ros2_upstream)  
 - [Marvelmind RTLS msg](https://github.com/MarvelmindRobotics/marvelmind_ros2_msgs_upstream)  
 - [RealSense Driver](https://github.com/IntelRealSense/realsense-ros)  

Main GitHub:  
 - [autonomous-lab](https://github.com/dedelstein/autonomous-lab)

## Setup
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

### Setting up the Test Bench with Docker
```bash
# Add docker to xhost
sudo xhost +local:docker

# Clone the primary git repo
git clone https://github.com/dedelstein/autonomous-lab

# Enter the repo root and initialize git submodules (defined in autonomous-lab/.gitmodules)
cd autonomous-lab
git submodule update --init

# Build the docker images (this takes time)
# If you have problems try adding --no-cache to the end of the command
sudo docker compose build ros2_bridge
sudo docker compose build ros2_master

# Mount the main docker image using rocker
# NB: --privileged gives us access to ports (usb, etc.), there's fancier and more
# secure ways to do this if someone has the time or inclination
sudo rocker --x11 --privileged -v .:/code ros2_master bash
# OR
# Spin up and execute image using docker compose
sudo docker compose up -d ros2_master
sudo docker exec -it ros2_master bash

# You should be in the /code directory in the docker container, which is a mountpoint for your local autonomous-lab folder

# If you need to, mount the ros1_bridge for ROS1-ROS2 communication in a separate terminal window
# ( NB - ros1_bridge waits for a ROS 1 master node at the default ROS Master URI http://localhost:11311 )
sudo docker compose up ros2_bridge
```

## Usage

### Test Bench ROS Nodes
```bash
# Navigate to the test bench workspace and build the workspace if not already done
cd /code/ROS2_Master/test_bench_ws  && ./init_ws.sh
# source the local overlay
source install/setup.bash

# Launch the following nodes from separate terminal windows
# (convenient to use tmux with multiple panes in this case)
# NB as of 9.4.2024 FLIR and RealSense have some kind of fatal conflict
# -- no solution found yet besides rebooting and only running one or the other
ros2 launch test_bench test_bench.launch.py
ros2 launch test_bench FLIR.launch.py
ros2 launch sick_scan_xd sick_mrs_6xxx.launch.py hostname:=192.168.1.18 frame_id:=LIDAR
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true
ros2 launch bno055 bno055.launch.py
ros2 launch marvelmind_ros marvelmind_ros.launch.py

# To end any particular node, just type CTRL-c in a window with an active node
```

## rviz2
```bash
# Launch rviz2 from test_bench_ws and load the custom setup (this setup may need tweaking)
rviz2 -d ./test-bench.rviz
```

## Documentation

### How Does This Work?

The [Dockerfile](https://github.com/dedelstein/autonomous-lab/blob/main/Dockerfile) in the [github project](https://github.com/dedelstein/autonomous-lab) includes a set of instructions to install necessary programs and the workspace into a docker container.  This container can be mounted using rocker, which allows for gui interactivity and use of nvidia drivers.  We have a base ROS install on our host volume, and then we have an 'overlay' in our test_bench_ws workspace.  If you are using the docker container, this is at /autonomous-lab/test_bench_ws.  This workspace contains three packages (so far) located in the test_bench_ws/src/ directory: test_bench, sick_scan_xd, and libsick_ldmrs.  libsick_ldmrs is a support package for sick_scan_xd, which is the driver for our SICK LIDAR.  test_bench includes launchers for the FLIR camera and our robot_state_publisher, which uses [URDF](#urdf) files to publish the relationship between various frames in the test bench.  Many of the drivers make use of custom parameter .yaml files located in ROS2_Master/sensor_params.

#### Parameter Files

In ROS2_Master/sensor_params there should be symlinks for the parameter files for most of the above described nodes once the workspace has been built with colcon.  Sometimes you need to change something in here like /dev/ttyACM1 vs /dev/ttyACM0, etc.


#### Marvelmind RTLS

First, you need to run the Marvelmind dashboard.  This can be done like so:
```bash
cd /code/Marvelmind_RTLS/
./dashboard.sh
```
Next you can set up the local map and then run the MarvelMind ROS2 Node. To understand how to use the marvelmind stuff, rtfm.  Sometimes it's necessary to unplug the IMU and the base station from USB and just plug in the modem first. The existing sensors are all mapped to device 2-6, with device 5 as the 'base hedgehog'.

#### URDF

The Unified Robot Description Format for our robot.  You can look at the files in test_bench_ws/src/test_bench/urdf/ folder or [here](https://github.com/dedelstein/autonomous-lab/tree/main/ROS2_Master/test_bench_ws/src/test_bench/urdf) to understand the current structure.  The files use [XACRO](https://docs.ros.org/en/iron/Tutorials/Intermediate/URDF/Using-Xacro-to-Clean-Up-a-URDF-File.html), which is just a convenient tool to add a few variables to [urdf](https://docs.ros.org/en/iron/Tutorials/Intermediate/URDF/Building-a-Visual-Robot-Model-with-URDF-from-Scratch.html) files, which are really just xml with a few conventions.  The most important thing here is the joint structure, which we refer to in our launch files, as we need spatial context for different data sources.  ![alt text](https://raw.githubusercontent.com/dedelstein/autonomous-lab/main/docs/images/tf_diag.png "TF2 frame diagram").  

#### Packages, Nodes & Launchers

ROS operates with different Nodes, that communicate with each other via messages that are divided into topics.  These nodes are launched with [command-line arguments](#test-bench-ros-nodes), which use launch files contained within each package.  A launch file should contain some snippet that looks like the following:
```python
def generate_launch_description():

    flir_node = Node(
        package = "spinnaker_camera_driver",
        executable = "camera_driver_node",
        output='screen',
        namespace='camera',
        # The 'parameters' block below can contain predefined parameters for the node to be launched
        parameters=[
            example_params,
            {
            'parameter_file': FLIR_params,
            'camera_type': 'chameleon',
            'serial_number': '20073275',
            'frame_id': 'FLIR'
            }]
        )
    return LaunchDescription([
        flir_node
    ])
```
We can also pass parameters to the launch file in our command-line argument.  These launch files create a node that publishes to a topic.  For example, our robot publishes data to the /cloud topic using sick_scan_xd, and we launch this driver with reference to the frame_id:=LIDAR, so that our /cloud topic adds the frame information from the robot_state_publisher node to its message header, which lets us use it in rviz.

### Current ROS structure
![alt text](https://raw.githubusercontent.com/dedelstein/autonomous-lab/main/docs/images/rqt_graph_26_2_2024.png "rqt_graph diagram")
