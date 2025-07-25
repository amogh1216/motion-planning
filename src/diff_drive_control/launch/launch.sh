#!/bin/bash

# Source ROS 2 and your workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

# Start the first tab with RViz launch
gnome-terminal \
  --tab --title="RViz" -e "bash -c 'ros2 launch diff_drive_control rviz_rwd.launch.py; exec bash'" \
  --tab --title="Localization" -e "bash -c 'sleep 7; ros2 run diff_drive_control localization_node; exec bash'" \
  --tab --title="Diff Drive Brain" -e "bash -c 'sleep 7; ros2 run diff_drive_control diff_drive_brain; exec bash'" \
  --tab --title="Diff Drive Brain Params" -e "bash -c 'sleep 9; ros2 param load /diff_drive_brain src/diff_drive_control/config/diff_drive_brain.yaml; exec bash'"