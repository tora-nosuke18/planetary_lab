import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    visual_odom_cmd = get_package_share_directory('visual_odom')
    navigation_cmd = get_package_share_directory('navigation')
    simulation_cmd = get_package_share_directory('rover_description')

    return LaunchDescription([
            
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(simulation_cmd, 'launch', 'gazebo_world.launch.py')
            )
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(visual_odom_cmd, 'launch', 'gazebo_vslam.launch.py')
            )
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(navigation_cmd, 'launch', 'navigation.launch.py')
            )
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(simulation_cmd, 'launch', 'nav2_rviz.launch.py')
            )
        ),
    ])
