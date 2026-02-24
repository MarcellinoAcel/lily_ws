import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    gazebo_launch = PathJoinSubstitution(
        [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']
    )

    urdf_path = PathJoinSubstitution(
        [FindPackageShare("lily_description"), "urdf", "robot.urdf.xacro"]
    )

    world_path = PathJoinSubstitution(
        [FindPackageShare('lily_description'), 'worlds', 'turtlebot3_world.sdf']
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'urdf',
            default_value=urdf_path,
            description='URDF file'
        ),

        DeclareLaunchArgument(
            'world',
            default_value=world_path,
            description='World file'
        ),

        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_z', default_value='0.1'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),

        # Launch Gazebo ONCE
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                'gz_args': [LaunchConfiguration('world'), ' -r']
            }.items()
        ),

        # Spawn robot
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

        # Bridge only required topics
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
                "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            ],
            output="screen"
        ),
    ])