from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():

    diff_drive_pkg = FindPackageShare('diff_drive_robot')
    nav2_pkg = FindPackageShare('nav2_bringup')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                nav2_pkg,
                'launch',
                'bringup_launch.py'
            ])
        ),
        launch_arguments={
            'map': '/home/pg/ros2_ws/src/diff_drive_robot/maps/my_map6.yaml',
            'use_sim_time': use_sim_time,
            'params_file': PathJoinSubstitution([
                diff_drive_pkg,
                'config',
                'nav2_params.yaml'
            ]),
            'autostart': 'true'
        }.items()
    )

    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d',
            PathJoinSubstitution([
                diff_drive_pkg,
                'rviz',
                'nav2.rviz'
            ])
    ],
    parameters=[{'use_sim_time': use_sim_time}],
    output='screen'
)

    return LaunchDescription([
        declare_use_sim_time,
        nav2_launch,
        rviz_node,
    ])