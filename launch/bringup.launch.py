from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_sim_time = True

    diff_drive_pkg = FindPackageShare('diff_drive_robot')
    nav2_pkg = FindPackageShare('nav2_bringup')

    # 1Robot + mapping
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                diff_drive_pkg,
                'launch',
                'robot.launch.py'
            ])
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Nav2 bringup
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                nav2_pkg,
                'launch',
                'bringup_launch.py'
            ])
        ),
        launch_arguments={
            'map': '/home/pg/my_map.yaml',
            'use_sim_time': 'true',
             'params_file': PathJoinSubstitution([
                diff_drive_pkg,
                'config',
                'nav2_params.yaml'
                ])
        }.items()
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d',
            '/opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        robot_launch,
        nav2_launch,
        rviz,
    ])
