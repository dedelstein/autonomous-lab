#! /bin/bash

# This script is used to initialize the workspace for the first time
# NB libsick_ldmrs must be built and sourced before sick_scan otherwise we could just do colcon build --cmake-args "-DROS_VERSION=2".
# Someone could probably fix this.  someone else.

colcon build --packages-select libsick_ldmrs --event-handlers console_direct+ && \
source /opt/ros/iron/setup.bash && \
source ./install/setup.bash && \
colcon build --packages-select sick_scan_xd --cmake-args " -DROS_VERSION=2" --event-handlers console_direct+ && \
source ./install/setup.bash && \
colcon build --packages-select test_bench && \
source ./install/setup.bash && \
colcon build --packages-select marvelmind_ros2_msgs && \
colcon build --packages-select marvelmind_ros2 && \
source ./install/setup.bash
