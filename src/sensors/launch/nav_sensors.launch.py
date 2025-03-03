import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # Arguments for the realsense2_camera launch
    args_rs = {
        'enable_fisheye': 'false',
        'enable_depth': 'true',
        'align_depth.enable': 'true',
        'enable_color': 'true',
        'enable_infra1': 'false',
        'enable_infra2': 'false',
        'enable_gyro': 'true' ,
        'enable_accel':'true' ,
        'unite_imu_method':'1',
    }

    imu_filter_params={
        'stateless': False,
        'use_mag': False,
        'publish_tf': True,
        'fixed_frame': "odom",
        'world_frame': "enu",
    }

    # Convert args_rs to the appropriate format for launch_arguments
    launch_arguments = [(key, value) for key, value in args_rs.items()]

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')]),
            launch_arguments=launch_arguments
        ),

        Node(
                package='imu_filter_madgwick',
                executable='imu_filter_madgwick_node',
                name='imu_filter',
                output='screen',
                parameters=[imu_filter_params],
                remappings=[('imu/data_raw','/camera/camera/imu')]
        ),

    ])
