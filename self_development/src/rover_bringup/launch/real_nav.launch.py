import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    navigation_cmd = get_package_share_directory('rover_navigation')
    simulation_cmd = get_package_share_directory('rover_simulation')
    controller_cmd = get_package_share_directory('rover_controller')

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(navigation_cmd, 'launch', 'rgbd_real.launch.py')
            )
        ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(simulation_cmd, 'launch', 'display.launch.py')
        #     )
        # ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(controller_cmd, 'launch', 'simple_run.launch.py')
            )
        )
    ])
