import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    visual_odom_cmd = get_package_share_directory('visual_odom')
    navigation_cmd = get_package_share_directory('navigation')
    sensors_cmd = get_package_share_directory('sensors')

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sensors_cmd, 'launch', 'nav_sensors.launch.py')
            )
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(visual_odom_cmd, 'launch', 'rgbd_real.launch.py')
            )
        ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(navigation_cmd, 'launch', 'navigation.launch.py')
        #     )
        # ),
    ])
