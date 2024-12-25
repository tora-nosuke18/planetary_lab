import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    params_file = LaunchConfiguration(
        'params', default=[os.path.join(
                get_package_share_directory('navigation'), 'params'),
                           '/navigation_params.yaml']
    )

    navigation_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('nav2_bringup'), 'launch'),
                '/navigation_launch.py']),
            launch_arguments ={'params_file' :[params_file]}.items()
        )
    
    slam = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('slam_toolbox'), 'launch'),
                '/online_async_launch.py'])
        )
    
    localization  =  Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("robot_localization"), 'params', 'ekf.yaml')],
           )
           
    ld.add_action(navigation_launch)
    ld.add_action(localization)

    return ld
