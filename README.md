# motion-planning
ROS2 + Gazebo simulation environment of high-level controls, localization, and motion planning for a differential drive robot. Pyqt5 visualizer for path generation, path following, and robot tracking. Tested with Ubuntu 22.04 machine on ROS Humble.

## Controls

The controls system supports different control strategies for moving the robot to a target pose: PID control, PID profiling ( acceleration and velocity constraints), and (trapezoidal time-based) motion profiling. Since the robot is non-holonomic, its path is separated into a turn and forward movement (controlled by a FSM).

The control mode can be selected at runtime using the `control_mode` parameter (`pid`, `pid_profiler`, or `motion_profile`). See `config.yaml`.

### Running

1. Build package: `colcon build` or `colcon build --packages-select diff_drive_control`
2. Run launch file: `sh src/diff_drive_control/launch/launch_ctrl.sh`. This file will launch Gazebo, RViz, and terminal tabs for debugging RViz, the localization node, and the brain node. It also loads parameters from the config file into the localization and brain nodes.
3. Go to the RViz window. Use 2D Goal Pose to set the target pose for the robot to navigate to.

## Path Following

For path following, the repository implements the Pure Pursuit algorithm integrated with a custom UI, allowing for interactive path creation and testing. See `pure_pursuit.py` for tuning parameters.

### Running
1. Build package: `colcon build` or `colcon build --packages-select diff_drive_control`
2. Run tracking simulator: `python3 ui/path_track/path_tracking_simulator.py`
3. Run launch file: `sh src/diff_drive_control/launch/launch_pth.sh`. This file will launch Gazebo, RViz, and terminal tabs for debugging Rviz, the localization node, and the pure pursuit node. It also loads parameters from the config file into the localization node.
4. Navigate to the simulator window. Draw your desired path. Click publish path for the robot to start following the desired path. 

## UI Add-on

A key feature of this project is the PyQt5-based visualizer UI, inspired by [this video](https://www.youtube.com/watch?v=u54WAlAewMU). This is useful in my machine without a GPU where I can not simulate crowded Gazebo scenes but still want to get the robot's true pose via the Gazebo physics engine. The UI makes it so I can use the robot's true position to create custom 2D environments to path plan under. Additionally, the UI will include 2D LiDAR (in development) for robot SLAM.

### Running Visualizer

To run the visualizer that purely tracks robot state: `python3 ui/path_vis/path_visualization_simulator.py`
The visualizer reads the topic `/get_pose` published by the localization node to paint the robot position on the screen.

## Localization

Right now the localization node estimates the robot's position using two different techniques: odometry and IMU. Odometry readings are calculated by the DiffDriveController automatically while the IMU readings are reconstructed from linear acceleration and angular velocity readings. The node also reads in the true robot pose from Gazebo’s transform messages for perfect localization. 

The localization mode can be selected via a ROS2 parameter (`mode`: `odom` or `truth`). See `config.yaml`. The node subscribes to odometry, IMU, and Gazebo transform topics, and publishes the estimated pose on `/get_pose`.

## Path Planning

In development with UI tool.
