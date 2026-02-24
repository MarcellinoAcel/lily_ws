from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition


def generate_launch_description():

    use_sim_time = True

    gazebo_launch_path = PathJoinSubstitution(
        [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']
    )

    world_path = PathJoinSubstitution([
        FindPackageShare('lily_description'),
        'worlds',
        'turtlebot3_world.sdf'
    ])

    urdf_path = PathJoinSubstitution([
        FindPackageShare("lily_description"),
        "urdf",
        "robot.urdf.xacro"
    ])

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('lily_description'), 'launch', 'description.launch.py']
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare("lily_description"), "rviz", "model_description.rviz"]
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            name='spawn_x',
            default_value='0.0'
        ),

        DeclareLaunchArgument(
            name='spawn_y',
            default_value='0.0'
        ),

        DeclareLaunchArgument(
            name='spawn_z',
            default_value='0.0'
        ),

        DeclareLaunchArgument(
            name='spawn_yaw',
            default_value='0.0'
        ),

        # Start Gazebo ONCE
        ExecuteProcess(
            cmd=[
                'gz', 'sim',
                '-r',        # run immediately
                world_path
            ],
            output='screen'
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

        # Bridge
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

        # Robot description
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path),
            launch_arguments={
                'use_sim_time': str(use_sim_time),
                'publish_joints_gui': 'false',
                'urdf': urdf_path
            }.items()
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        )
    ])