from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    urdf_path = PathJoinSubstitution(
        [FindPackageShare("lily_description"), "urdf", "robot.urdf.xacro"]
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare("lily_description"), "rviz", "model_description.rviz"]
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            name='urdf',
            default_value=urdf_path,
            description='URDF path'
        ),

        DeclareLaunchArgument(
            name='publish_joints_gui',
            default_value='true',
            description='Use joint_state_publisher_gui'
        ),

        DeclareLaunchArgument(
            name='rviz',
            default_value='false',
            description='Run RViz'
        ),

        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        # Non-GUI version
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            condition=UnlessCondition(LaunchConfiguration("publish_joints_gui")),
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),

        # GUI version
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(LaunchConfiguration("publish_joints_gui")),
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': Command([
                    'xacro ',
                    LaunchConfiguration('urdf')
                ])
            }]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),
    ])