#!/bin/bash

# Source ROS 2 and your workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

# Start the first tab with RViz launch
gnome-terminal \
  --tab --title="RViz" -e "bash -c 'ros2 launch diff_drive_control rviz_rwd.launch.py; exec bash'" \
  --tab --title="Localization" -e "bash -c 'sleep 5; ros2 run diff_drive_control localization_node; ros2 param load 
                        /localization_node src/diff_drive_control/config/config.yaml; exec bash'" \
  --tab --title="Path Following" -e "bash -c 'sleep 5; ros2 run diff_drive_control pure_pursuit.py; exec bash'"

