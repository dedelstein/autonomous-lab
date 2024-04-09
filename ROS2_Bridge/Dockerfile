FROM ros:foxy-ros-base-focal

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list

ENV ROS1_DISTRO noetic
ENV ROS2_DISTRO foxy

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-comm=1.16.0-1* \
    ros-noetic-roscpp-tutorials=0.10.2-1* \
    ros-noetic-rospy-tutorials=0.10.2-1* \
    && rm -rf /var/lib/apt/lists/*

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-foxy-ros1-bridge=0.9.7-1* \
    ros-foxy-demo-nodes-cpp=0.9.4-1* \
    ros-foxy-demo-nodes-py=0.9.4-1* \
    && rm -rf /var/lib/apt/lists/*

COPY ./bridge_entry.sh /

ENTRYPOINT ["/bridge_entry.sh"]
