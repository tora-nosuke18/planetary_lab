import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    stereo_odometry_params = {
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'publish_tf': True,
        'queue_size': 10,
        'approx_sync': False,
        'use_sim_time' : True,
        # 'wait_for_transform': True,
        'Stereo/MaxDisparity': "256",
        'Stereo/MinDisparity' : "0" ,
        'OdomF2M/MaxSize': '1000',
        'GFTT/MinDistance': '10',
        'Odom/MinInliers' : "50"

    }

    pkg_stereo_image_proc = get_package_share_directory(
        'stereo_image_proc')

    # Paths
    stereo_image_proc_launch = PathJoinSubstitution(
        [pkg_stereo_image_proc, 'launch', 'stereo_image_proc.launch.py'])

    rtabmap_params = {
        'frame_id': 'base_link',
        'subscribe_stereo': True,
        'subscribe_depth': False,
        'approx_sync': False,
        'wait_for_transform_duration': 0.2,
        'map_always_update': True,
        'queue_size': 10,
        'use_sim_time' : True,

        'GridGlobal/MinSize': '100',
        # 'Vis/MinInliers': 12,
        # 'Mem/UseOdomGravity': True,
        # 'Optimizer/GravitySigma': 0.3,
        # 'Vis/MaxDepth': 3.5,
        # 'Vis/FeatureType': 8
    }

    remappings = [
        ('odom', '/visual_odom'),
        ('left/image_rect', '/infra1/image_raw'),
        ('left/camera_info', '/infra1/camera_info'),
        ('right/image_rect', '/infra2/image_raw'),
        ('right/camera_info', '/infra2/camera_info')
    ]


    return LaunchDescription([      
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([stereo_image_proc_launch]),
            launch_arguments=[
                # ('left_namespace', 'stereo_camera/left'),
                # ('right_namespace', 'stereo_camera/right'),
                ('disparity_range', '256'),
            ]
        ),
    
        Node(
            package='rtabmap_odom', executable='stereo_odometry', 
            parameters=[stereo_odometry_params],  # List format
            remappings=remappings
        ),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[rtabmap_params],  # List format
            remappings=remappings,
        ),
    ])
