# motion-planning
ROS2 + Gazebo simulation environment of high-level controls, localization, and motion planning for a differential drive robot. Pyqt5 visualizer for path generation, path following, and robot tracking. Tested with Ubuntu 22.04 machine on ROS Humble.

## Controls

The controls system supports different control strategies for moving the robot to a target pose:

- **PID Control:** Uses a PID controller to first turn the robot to face the goal, then move forward. This basic motion planning is commanded by an FSM. The controller transitions through states: Turning, Stabilizing, Moving, and Stopping. Parameters for both linear and angular PID can be tuned via parameters. See `config.yaml`.
- **PID Profiler Control:** Extends PID control with motion profiling for smoother velocity transitions, using both linear and angular velocity profiles.
- **Motion Profile Control:** Uses trapezoidal motion profiles for both turning and moving forward, generating smooth velocity commands at each phase. This is method is significantly more innacurate than the PID, as the profile is generated and followed under strict time requirements. 

The control mode can be selected at runtime using the `control_mode` parameter (`pid`, `pid_profiler`, or `motion_profile`). See `config.yaml`. The system receives the robot’s current pose and goal pose via ROS2 topics, and publishes velocity commands (TwistStamped) accordingly. State transitions and controller status are logged for transparency and debugging.

### Running

1. Build package: `colcon build` or `colcon build --packages-select diff_drive_control`
2. Run launch file: `sh src/diff_drive_control/launch/launch_ctrl.sh`. This file will launch Gazebo, RViz, and terminal tabs for debugging Rviz, the localization node, and the brain node. It also loads parameters from the config file into the localization and brain nodes.

## Path Following

For path following, the repository implements the Pure Pursuit algorithm integrated with a custom UI, allowing for interactive path creation and testing. See `pure_pursuit.py` for tuning parameters.

### Running
1. Build package: `colcon build` or `colcon build --packages-select diff_drive_control`
2. Run tracking simulator: `python3 ui/path_track/path_tracking_simulator.py`
3. Run launch file: `sh src/diff_drive_control/launch/launch_pth.sh`. This file will launch Gazebo, RViz, and terminal tabs for debugging Rviz, the localization node, and the pure pursuit node. It also loads parameters from the config file into the localization node.
4. Navigate to the simulator window. Draw your desired path. Click publish path for the robot to start following the desired path. 

## Visualization

A key feature of this project is the PyQt5-based visualizer UI, inspired by [this video](https://www.youtube.com/watch?v=u54WAlAewMU). Unlike traditional approaches that rely on Nav2 maps and Gazebo for scene setup, this UI allows for rapid prototyping and flexible scene creation (in development). Importantly, This makes it easy to test scenarios that would be too computationally expensive to render in Gazebo, while still allowing the robot to be influenced by the Gazebo physics system (or any other custom physics engine). 

### Running Visualizer

To run the visualizer that purely tracks robot state: `python3 ui/path_vis/path_visualization_simulator.py`
The visualizer reads the topic `/get_pose` published by the localization node to paint the robot position on the screen.

## Localization

The localization node estimates the robot’s position using multiple techniques and publishes the current pose for use by controllers and planners. It supports:

- **Odometry-based Localization:** Uses wheel odometry from the differential drive controller to estimate position and heading. The controller plugin handles position calculation based on velocity (TwistStamped) commands. This odometry reading is exactly how the robot moves in RViz.
- **IMU-based Localization:** Integrates linear acceleration from the IMU sensor (transformed into the world frame using the robot’s heading) to estimate velocity and position. Integrates angular velocity to estimate angular position. Highly inaccurate reading when on paths that require turning.
- **Ground Truth (Gazebo):** The node can use the true robot pose from Gazebo’s transform messages for perfect localization during simulation. Differs from odometry.

The localization mode can be selected via a ROS2 parameter (`mode`: `odom` or `truth`). See `config.yaml`. The node subscribes to odometry, IMU, and Gazebo transform topics, and publishes the estimated pose on `/get_pose`.

## Path Planning

In development. Will involve development of visualizer tool.