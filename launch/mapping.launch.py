import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    package_name = 'diff_drive_robot'

    # Declare use_sim_time (IMPORTANT)
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    # SLAM toolbox params file
    slam_params_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'slam_toolbox_mapping.yaml'
    )

    # Include slam_toolbox launch
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file,
        }.items()
    )

    # RViz config file
    rviz_config_file = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        'mapping.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        declare_use_sim_time,
        slam_toolbox_launch,
        rviz_node,
    ])
