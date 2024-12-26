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
        ('left/image_rect', '/stereo_camera/infra1/image_rect'),
        ('left/camera_info', '/stereo_camera/infra1/camera_info'),
        ('right/image_rect', '/stereo_camera/infra2/image_rect'),
        ('right/camera_info', '/stereo_camera/infra2/camera_info')
    ]


    return LaunchDescription([      
        Node(
            package='v4l2_camera', executable='v4l2_camera_node',name='left_cam_node',
            namespace='/web_camera/left',
            parameters=[{'video_device': '/dev/video0'}]
            ),

        Node(
            package='v4l2_camera', executable='v4l2_camera_node',name='right_cam_node',
            namespace='/web_camera/right',
            parameters=[{'video_device': '/dev/video2'}]
            ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([stereo_image_proc_launch]),
            launch_arguments=[
                ('left_namespace', 'web_camera/left'),
                ('right_namespace', 'web_camera/right'),
                ('disparity_range', '256'),
            ]
        ),
    
        # Node(
        #     package='rtabmap_odom', executable='stereo_odometry', 
        #     parameters=[stereo_odometry_params],  # List format
        #     remappings=remappings
        # ),

        # Node(
        #     package='rtabmap_slam', executable='rtabmap', output='screen',
        #     parameters=[rtabmap_params],  # List format
        #     remappings=remappings,
        # ),
    ])
