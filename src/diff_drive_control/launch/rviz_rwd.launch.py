# adapted from https://github.com/art-e-fact/navigation2_ignition_gazebo_example/blob/26b1d74431d2d6130d23880a5f40416abfe4cfc7/src/sam_bot_nav2_gz/launch/display.launch.py#L123C5-L136C6
# goated
import launch
from launch.actions import (
    ExecuteProcess,
    DeclareLaunchArgument,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    NotSubstitution,
    AndSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros
import os
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(
        package="diff_drive_control"
    ).find("diff_drive_control")
    default_model_path = os.path.join(
        pkg_share, "urdf/rwd.xacro"
    )
    default_rviz_config_path = os.path.join(pkg_share, "config/robot-config.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_localization = LaunchConfiguration("use_localization")
    use_rviz = LaunchConfiguration("use_rviz")
    log_level = LaunchConfiguration("log_level")
    gz_verbosity = LaunchConfiguration("gz_verbosity")
    run_headless = LaunchConfiguration("run_headless")
    world_file_name = LaunchConfiguration("world_file")
    gz_models_path = ":".join([pkg_share, os.path.join(pkg_share, "urdf")])
    #gz_models_path = os.path.join(pkg_share, "models")
    world_path = PathJoinSubstitution([pkg_share, "config", "my_world.sdf"])

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[                                # LaunchConfiguration("model")
            {"robot_description": Command(["xacro ", default_model_path])}
        ],
    )

    rviz_node = Node(
        condition=IfCondition(AndSubstitution(NotSubstitution(run_headless), use_rviz)),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )
    
    # gazebo have to be executed with shell=False, or test_launch won't terminate it
    #   see: https://github.com/ros2/launch/issues/545
    # This code is form taken ros_gz_sim and modified to work with shell=False
    #   see: https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_sim/launch/gz_sim.launch.py.in
    gz_env = {'GZ_SIM_SYSTEM_PLUGIN_PATH':
           ':'.join([os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', default=''),
                     os.environ.get('LD_LIBRARY_PATH', default='')]),
           'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':  # TODO(CH3): To support pre-garden. Deprecated.
                      ':'.join([os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', default=''),
                                os.environ.get('LD_LIBRARY_PATH', default='')])}
    gazebo = [
        ExecuteProcess(
            condition=launch.conditions.IfCondition(run_headless),
            cmd=['ruby', FindExecutable(name="ign"), 'gazebo',  '-r', '-v', gz_verbosity, '-s', '--headless-rendering', world_path],
            output='screen',
            additional_env=gz_env, # type: ignore
            shell=False,
        ),
        ExecuteProcess(
            condition=launch.conditions.UnlessCondition(run_headless),
            cmd=['ruby', FindExecutable(name="ign"), 'gazebo',  '-r', '-v', gz_verbosity, world_path],
            output='screen',
            additional_env=gz_env, # type: ignore
            shell=False,
        )
    ]

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",
            "rwd_bot",
            "-topic",
            "robot_description",
            "-z",
            "1.0",
            "-x",
            "0.0",
            "--ros-args",
            "--log-level",
            log_level,
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            "/world/my_world/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"
        ],
        output="screen",
    )


    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('diff_drive_control'),
            'config',
            'rwd_diff_controller.yaml',
        ]
    )

    diff_drive_base_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'rwd_diff_controller',
            '--param-file',
            robot_controllers
        ],
    )

    localization = Node (
        package='diff_drive_control',
        executable='localization_node',
    )

    diff_drive_brain = Node (
        package='diff_drive_control',
        executable='diff_drive_brain',
        arguments=[
            'rwd_pid',
            '--param-file',
            robot_controllers
        ]
    )

    plotjuggler_node = Node(
        package="plotjuggler",
        executable="plotjuggler",
        name="plotjuggler",
        output="screen"
    )

    pure_pursuit_node = Node(
        package="diff_drive_control",
        executable="pure_pursuit.py",
        name="pure_pursuit",
        output="screen"
    )

    return launch.LaunchDescription(
        [
            SetEnvironmentVariable(
                name="IGN_GAZEBO_RESOURCE_PATH",
                value=gz_models_path,
            ),
            DeclareLaunchArgument(
                name="model",
                default_value=default_model_path,
                description="Absolute path to robot urdf file",
            ),
            DeclareLaunchArgument(
                name="use_rviz",
                default_value="True",
                description="Start RViz",
            ),
            DeclareLaunchArgument(
                name="run_headless",
                default_value="False",
                description="Start GZ in hedless mode and don't start RViz (overrides use_rviz)",
            ),
            DeclareLaunchArgument(
                name="world_file",
                default_value="empty.sdf",
            ),
            DeclareLaunchArgument(
                name="rvizconfig",
                default_value=default_rviz_config_path,
                description="Absolute path to rviz config file",
            ),
            DeclareLaunchArgument(
                name="use_sim_time",
                default_value="True",
                description="Flag to enable use_sim_time",
            ),
            DeclareLaunchArgument(
                name="use_localization",
                default_value="True",
                description="Use EKF to estimagte odom->base_link transform from IMU + wheel odometry",
            ),
            DeclareLaunchArgument(
                "gz_verbosity",
                default_value="3",
                description="Verbosity level for Ignition Gazebo (0~4).",
            ),
            DeclareLaunchArgument(
                "gz_args",
                default_value="",
                description="Extra args for Gazebo (ie. '-s' for running headless)",
            ),
            DeclareLaunchArgument(
                name="log_level",
                default_value="warn",
                description="The level of logging that is applied to all ROS 2 nodes launched by this script.",
            ),
            bridge,
            robot_state_publisher_node,
            spawn_entity,
            diff_drive_base_controller_spawner,
            # rviz_node,
            joint_state_broadcaster_spawner,
            # pure_pursuit_node
        ] + gazebo
    )