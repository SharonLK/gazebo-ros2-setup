from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ---------------------------------------------------------
    # Paths
    # ---------------------------------------------------------

    robot_xacro = PathJoinSubstitution(
        [
            FindPackageShare('robot_description'),
            'urdf',
            'robot.urdf.xacro',
        ]
    )

    world = PathJoinSubstitution(
        [
            FindPackageShare('robot_gazebo'),
            'worlds',
            'lab.sdf',
        ]
    )

    controllers_file = PathJoinSubstitution(
        [
            FindPackageShare('robot_control'),
            'config',
            'controllers.yaml',
        ]
    )

    # ---------------------------------------------------------
    # Robot description
    # ---------------------------------------------------------

    robot_description = {
        'robot_description': Command(
            [
                FindExecutable(name='xacro'),
                ' ',
                robot_xacro,
            ]
        )
    }

    # ---------------------------------------------------------
    # Gazebo
    # ---------------------------------------------------------

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare('ros_gz_sim'),
                    'launch',
                    'gz_sim.launch.py',
                ]
            )
        ),
        launch_arguments={
            'gz_args': [
                '-r -v 2 ',
                world,
            ],
        }.items(),
    )

    # ---------------------------------------------------------
    # Robot State Publisher
    # ---------------------------------------------------------

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {
                'use_sim_time': True,
            },
        ],
    )

    # ---------------------------------------------------------
    # Spawn robot into Gazebo
    # ---------------------------------------------------------

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic',
            'robot_description',
            '-name',
            'robot_lab',
            '-z',
            '0.02',
        ],
    )

    # ---------------------------------------------------------
    # ros2_control controller spawners
    # ---------------------------------------------------------

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
        ],
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'diff_drive_controller',
            '--controller-manager',
            '/controller_manager',
            '--param-file',
            controllers_file,
        ],
    )

    # ---------------------------------------------------------
    # Gazebo simulation clock -> ROS 2
    # ---------------------------------------------------------

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
    )

    # ---------------------------------------------------------
    # Gazebo simulation scan -> ROS 2
    # ---------------------------------------------------------

    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
    )

    # ---------------------------------------------------------
    # Startup sequencing
    #
    # 1. Spawn robot
    # 2. Start joint_state_broadcaster
    # 3. Start diff_drive_controller
    # ---------------------------------------------------------

    start_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                joint_state_broadcaster_spawner,
            ],
        )
    )

    start_diff_drive_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                diff_drive_controller_spawner,
            ],
        )
    )

    # ---------------------------------------------------------
    # Launch
    # ---------------------------------------------------------

    return LaunchDescription(
        [
            gazebo,
            robot_state_publisher,
            clock_bridge,
            lidar_bridge,
            start_joint_state_broadcaster,
            start_diff_drive_controller,
            spawn_robot,
        ]
    )
