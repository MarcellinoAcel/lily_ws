from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition


def generate_launch_description():

    use_sim_time = True

    world_path = PathJoinSubstitution([
        FindPackageShare('lily_description'),
        'worlds',
        'turtlebot3_world_1.sdf'
    ])

    urdf_path = PathJoinSubstitution([
        FindPackageShare("lily_description"),
        "urdf",
        "robot.urdf.xacro"
    ])

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('lily_description'), 'launch', 'description.launch.py']
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare("lily_description"), "rviz", "model_description.rviz"]
    )

    return LaunchDescription([

        DeclareLaunchArgument(name='spawn_x', default_value='0.0'),
        DeclareLaunchArgument(name='spawn_y', default_value='0.0'),
        DeclareLaunchArgument(name='spawn_z', default_value='0.0'),
        DeclareLaunchArgument(name='spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument(name='rviz', default_value='false'),

        ExecuteProcess(
            cmd=['gz', 'sim','--fullscreen', '-r', world_path],
            output='screen'
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'lily',
                '-x', LaunchConfiguration('spawn_x'),
                '-y', LaunchConfiguration('spawn_y'),
                '-z', LaunchConfiguration('spawn_z'),
                '-Y', LaunchConfiguration('spawn_yaw'),
            ]
        ),

        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
                "/odom/unfiltered@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
                "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
                "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            ],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path),
            launch_arguments={
                'use_sim_time': str(use_sim_time),
                'publish_joints_gui': 'false',
                'urdf': urdf_path
            }.items()
        ),

        # ✅ ros2_control_node REMOVED — gz_ros2_control plugin handles this inside Gazebo

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["leg_controller"],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )
    ])