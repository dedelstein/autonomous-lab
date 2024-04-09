#! /bin/bash

# This script is used to initialize the workspace for the first time

git submodule update --init && \
colcon build --packages-select libsick_ldmrs --event-handlers console_direct+ && \
source /opt/ros/iron/setup.bash && \
source ./install/setup.bash && \
colcon build --packages-select sick_scan_xd --cmake-args " -DROS_VERSION=2" --event-handlers console_direct+ && \
source ./install/setup.bash && \
colcon build --packages-select test_bench && \
source ./install/setup.bash
