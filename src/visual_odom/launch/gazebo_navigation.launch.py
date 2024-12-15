import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch_ros.actions import Node, SetRemap
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

    stereo_odometry_params = {
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'publish_tf': True,
        'queue_size': 10,
        'wait_for_transform': True,
        'Odom/MinInliers': 12
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
        'Vis/MinInliers': 12,
        'GridGlobal/MinSize': '100',
        'Mem/UseOdomGravity': True,
        'Optimizer/GravitySigma': 0.3,
        'Vis/MaxDepth': 3.5,
        'Vis/FeatureType': 8
    }

    remappings = [
        # ('odom', '/visual_odom'),
        ('left/image_rect', '/stereo_camera/infra1/image_rect'),
        ('left/camera_info', '/stereo_camera/infra1/camera_info'),
        ('right/image_rect', '/stereo_camera/infra2/image_rect'),
        ('right/camera_info', '/stereo_camera/infra2/camera_info')
    ]


    return LaunchDescription([
        # Run the ROS package stereo_image_proc for image rectification   
        GroupAction(
            actions=[

                SetRemap(src='camera_info',dst='camera_info_throttle'),
                SetRemap(src='camera_info',dst='camera_info_throttle'),

                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([stereo_image_proc_launch]),
                    launch_arguments=[
                        ('left_namespace', 'stereo_camera/infra1'),
                        ('right_namespace', 'stereo_camera/infra2'),
                        ('disparity_range', '128'),
                    ]
                ),
            ]
        ),

        Node(
            package='rtabmap_sync', executable='stereo_sync', output='screen',
            namespace='stereo_camera',
            remappings=[
                ('left/image_rect',   'left/image_rect'),
                ('right/image_rect',  'right/image_rect'),
                ('left/camera_info',  'left/camera_info_throttle'),
                ('right/camera_info', 'right/camera_info_throttle')]),

        # Nodes to launch       
        Node(
            package='rtabmap_odom', executable='stereo_odometry', output='screen',
            # parameters=[stereo_odometry_params],  # List format
            remappings=remappings
        ),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[rtabmap_params],  # List format
            # remappings=remappings,
        ),
    
        # Uncomment this if you need to use rtabmap_viz
        # Node(
        #     package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        #     parameters=[rtabmap_params],
        #     remappings=remappings
        # ),
    ])
