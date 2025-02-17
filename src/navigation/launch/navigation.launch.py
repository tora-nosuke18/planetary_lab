import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    nav2_params = LaunchConfiguration(
        'params', default=[os.path.join(
                get_package_share_directory('navigation'), 'params'),
                           '/navigation_params.yaml']
    )

    ekf_params = LaunchConfiguration(
        'params', default=[os.path.join(
                get_package_share_directory('navigation'), 'params'),
                           '/ekf.yaml']
    )
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')


    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('nav2_bringup'), 'launch'),
                '/navigation_launch.py']),
            launch_arguments ={'params_file' :[nav2_params],'use_sim_time': "True"}.items(),        # we must set use_sim_time to True
        ),
    
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params, {'use_sim_time': use_sim_time}],
           )
    ])
