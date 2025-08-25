# motion-planning
ROS2 + Gazebo simulation environment of high-level controls, localization, and motion planning for a differential drive robot. Pyqt5 visualizer for path generation, path following, and robot tracking. Tested with Ubuntu 22.04 on ROS Humble.

A key feature of this project is the PyQt5-based visualizer UI, inspired by [this video](https://www.youtube.com/watch?v=u54WAlAewMU). This is useful in machines like mine without a GPU where crowded Gazebo scenes are difficult to simulate. In this case, I use Gazebo to simulate wheel slippage of the robot, and I use the UI to handle scene development. This tool offers a lot of flexibility for development in localization, path generation, path following, and controls without having an intensive 3D environment.

## LiDAR, SLAM

2D LiDAR is simulated within `path_tracking_simulator.py`. Essentially, the program finds intersections between 'rays' that are extended from the robot's true position and obstacles drawn on the UI. These intersections form a point cloud. See `ui_simulator_params.yaml` to configure its parameters. Using localization from odometry, I take point clouds and map them relative to the odometry-localized position. Both the true (red cloud) and estimated (black cloud) maps are drawn on the UI.

1. Build package. `colcon build --packages-select diff_drive_control`.
2. Run launch file: `sh src/diff_drive_control/launch/launch_pth.sh`. Launches Gazebo and terminal tabs for debugging. 
3. Run simulator: `python3 ui/path_track/path_tracking_simulator.py`. Click Draw Obstacles to draw boundaries for the robot to map. Only supports straight lines now. Click Create Map for the robot to start mapping.

## Path Following

For path following, the repository implements the Pure Pursuit algorithm integrated with a custom UI, allowing for interactive path creation and testing. See `pure_pursuit.py` for tuning parameters.

### Running
1. Build package: `colcon build` or `colcon build --packages-select diff_drive_control`
2. Run tracking simulator: `python3 ui/path_track/path_tracking_simulator.py`
3. Run launch file: `sh src/diff_drive_control/launch/launch_pp.sh`. This file will launch Gazebo and terminal tabs for debugging Rviz, the localization node, and the pure pursuit node. It also loads parameters from the config file into the localization node.
4. Navigate to the simulator window. Draw your desired path. Click publish path for the robot to start following the desired path. 

### Running Visualizer

To run the visualizer that purely tracks robot state: `python3 ui/path_vis/path_visualization_simulator.py`
The visualizer reads the topic `/get_pose` published by the localization node to paint the robot position on the screen.

## Localization

Right now the localization node estimates the robot's position using two different techniques: odometry and IMU. Odometry readings are calculated by the DiffDriveController automatically while the IMU readings are reconstructed from linear acceleration and angular velocity readings. The node also reads in the true robot pose from Gazebo’s transform messages for perfect localization. 

The localization mode can be selected via a ROS2 parameter (`mode`: `odom` or `truth`). See `config.yaml`. The node subscribes to odometry, IMU, and Gazebo transform topics, and publishes the estimated pose on `/get_pose`.

## Controls

The controls system supports different control strategies for moving the robot to a target pose: PID control, PID profiling ( acceleration and velocity constraints), and (trapezoidal time-based) motion profiling. Since the robot is non-holonomic, its path is separated into a turn and forward movement (controlled by a FSM).

The control mode can be selected at runtime using the `control_mode` parameter (`pid`, `pid_profiler`, or `motion_profile`). See `config.yaml`.

### Running

1. Build package: `colcon build` or `colcon build --packages-select diff_drive_control`
2. Run launch file: `sh src/diff_drive_control/launch/launch_ctrl.sh`. This file will launch RViz, Gazebo and terminal tabs for the localization node and the brain node. It also loads parameters from the config file into the localization and brain nodes.
3. Go to the RViz window. Use 2D Goal Pose to set the target pose for the robot to navigate to.

## Path Planning

In development with UI tool.
