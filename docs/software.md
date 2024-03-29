# Overview of Software Architecture

OS: Ubuntu 22.04 LTS  
ROS Distro: [Iron](https://docs.ros.org/en/iron/Releases/Release-Iron-Irwini.html)  
  
Drivers:  
 - [sick_scan_xd](https://github.com/SICKAG/sick_scan_xd/)  
 - [spinnaker camera driver](https://index.ros.org/p/spinnaker_camera_driver/)  

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
# Build the docker image and create the volume (this takes time)
sudo docker image build --tag test-bench .
sudo docker volume create test-bench

# Mount the docker volume using rocker
# NB: --privileged gives us access to ports (usb, etc.), there's fancier and more
# secure ways to do this if someone has the time or inclination
sudo rocker --x11 --privileged test-bench bash
```

## Usage

### Test Bench ROS Nodes
```bash
# Navigate to the test bench workspace and source the overlay
cd autonomous-lab/test_bench_ws  && source install/setup.bash

# Launch the following nodes from separate terminal windows
# (convenient to use tmux with multiple panes in this case)
ros2 launch test_bench test_bench.launch.py
ros2 launch test_bench FLIR.launch.py
ros2 launch sick_scan_xd sick_mrs_6xxx.launch.py hostname:=192.168.1.18 frame_id:=LIDAR

# To end any particular node, just type CTRL-c in a window with an active node
```

## rviz2
```bash
# Launch rviz2 from test_bench_ws and load the custom setup
rviz2 -d ./test-bench.rviz
```

## Documentation

### How Does This Work?

The [Dockerfile](https://github.com/dedelstein/autonomous-lab/blob/main/Dockerfile) in the [github project](https://github.com/dedelstein/autonomous-lab) includes a set of instructions to install necessary programs and the workspace into a docker container.  This container can be mounted using rocker, which allows for gui interactivity and use of nvidia drivers.  We have a base ROS install on our host volume, and then we have an 'overlay' in our test_bench_ws workspace.  If you are using the docker container, this is at /autonomous-lab/test_bench_ws.  This workspace contains three packages (so far) located in the test_bench_ws/src/ directory: test_bench, sick_scan_xd, and libsick_ldmrs.  libsick_ldmrs is a support package for sick_scan_xd, which is the driver for our SICK LIDAR.  test_bench includes launchers for the FLIR camera and our robot_state_publisher, which uses [URDF](#urdf) files to publish the relationship between various frames in the test bench.

#### URDF

The Unified Robot Description Format for our robot.  You can look at the files in autonomous-lab/test_bench_ws/src/test_bench/urdf/ folder or [here](https://github.com/dedelstein/autonomous-lab/tree/main/test_bench_ws/src/test_bench/urdf) to understand the current structure.  The files use [XACRO](https://docs.ros.org/en/iron/Tutorials/Intermediate/URDF/Using-Xacro-to-Clean-Up-a-URDF-File.html), which is just a convenient tool to add a few variables to [urdf](https://docs.ros.org/en/iron/Tutorials/Intermediate/URDF/Building-a-Visual-Robot-Model-with-URDF-from-Scratch.html) files, which are really just xml with a few conventions.  The most important thing here is the joint structure, which we refer to in our launch files, as we need spatial context for different data sources.  ![alt text](https://raw.githubusercontent.com/dedelstein/autonomous-lab/main/docs/images/tf_diag.png "TF2 frame diagram").  

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
