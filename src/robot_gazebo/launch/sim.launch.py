from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Every robot exposes the same topic contract, whatever its controllers are
# called internally: /cmd_vel (in), /odom, /scan (out), /clock.
CONTROLLER_REMAPS = '-r diff_drive_controller/cmd_vel:=/cmd_vel -r diff_drive_controller/odom:=/odom'


def launch_behavior(context):
    behavior = LaunchConfiguration('behavior').perform(context)

    if not behavior:
        return []

    return [
        Node(
            package='robot_logic',
            executable=behavior,
            output='screen',
            parameters=[{'use_sim_time': True}],
        )
    ]


def generate_launch_description():
    robot = LaunchConfiguration('robot')

    # A robot is a self-contained folder in the `robots` package:
    #   <robot>/robot.urdf.xacro, <robot>/controllers.yaml, <robot>/bridges.yaml
    def robot_file(name):
        return PathJoinSubstitution(
            [FindPackageShare('robots'), robot, name],
        )

    robot_xacro = robot_file('robot.urdf.xacro')
    controllers_file = robot_file('controllers.yaml')
    bridges_file = robot_file('bridges.yaml')

    world = PathJoinSubstitution(
        [
            FindPackageShare('robot_gazebo'),
            'worlds',
            [LaunchConfiguration('world'), TextSubstitution(text='.sdf')],
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
            robot,
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
            '--controller-ros-args',
            CONTROLLER_REMAPS,
        ],
    )

    # ---------------------------------------------------------
    # Gazebo simulation clock -> ROS 2
    # All Gazebo <-> ROS topics for the robot are listed in its bridges.yaml
    # ---------------------------------------------------------
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[
            {
                'config_file': bridges_file,
                'use_sim_time': True,
            }
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
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    start_diff_drive_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )

    # ---------------------------------------------------------
    # Launch
    # ---------------------------------------------------------

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'robot',
                default_value='diffdrive_lidar',
                description='Robot folder name inside the `robots` package',
            ),
            DeclareLaunchArgument(
                'world',
                default_value='lab',
                description='World name in robot_gazebo/worlds (without .sdf)',
            ),
            DeclareLaunchArgument(
                'behavior',
                default_value='',
                description='robot_logic executable to start (e.g. drive_demo); empty = none',
            ),
            gazebo,
            robot_state_publisher,
            bridge,
            start_joint_state_broadcaster,
            start_diff_drive_controller,
            spawn_robot,
            OpaqueFunction(function=launch_behavior),
        ]
    )
