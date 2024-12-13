import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch_ros.actions import Node, SetRemap
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    stereo_odometry_params = {
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'publish_tf': True,
        'queue_size': 10,
        'wait_for_transform': True,
        'Odom/MinInliers': 12
    }

    rtabmap_params = {
        'frame_id': 'base_link',
        'subscribe_stereo': True,
        'subscribe_depth': False,
        'approx_sync': False,
        'wait_for_transform_duration': 0.2,
        'map_always_update': True,
        'queue_size': 10,
        'Vis/MinInliers': 12,
        'GridGlobal/MinSize': '100',
        'Mem/UseOdomGravity': True,
        'Optimizer/GravitySigma': 0.3,
        'Vis/MaxDepth': 3.5,
        'Vis/FeatureType': 8
    }

    remappings = [
        ('odom', 'visual/odom'),
        ('left/image_rect', 'infra1/image_raw'),
        ('left/camera_info', 'infra1/camera_info'),
        ('right/image_rect', 'infra2/image_raw'),
        ('right/camera_info', 'infra2/camera_info')
    ]

    return LaunchDescription([

        # Nodes to launch       
        Node(
            package='rtabmap_odom', executable='stereo_odometry', output='screen',
            # parameters=[stereo_odometry_params],  # List format
            remappings=remappings
        ),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[rtabmap_params],  # List format
            remappings=remappings,
        ),
    
        # Uncomment this if you need to use rtabmap_viz
        # Node(
        #     package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        #     parameters=[rtabmap_params],
        #     remappings=remappings
        # ),
    ])
