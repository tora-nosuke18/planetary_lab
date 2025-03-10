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
        'enable_depth': 'false',
        'enable_color': 'false',
        'enable_infra1': 'true',
        'enable_infra2': 'true'
    }

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
        # 'Vis/MinInliers': 12,
        # 'GridGlobal/MinSize': 100,
        # 'Mem/UseOdomGravity': True,
        # 'Optimizer/GravitySigma': 0.3,
        # 'Vis/MaxDepth': 3.5,
        # 'Vis/FeatureType': 8
    }

    remappings = [
        ('odom', '/visual_odom'),
        ('left/image_rect', '/camera/camera/infra1/image_rect_raw'),
        ('left/camera_info', '/camera/camera/infra1/camera_info'),
        ('right/image_rect', '/camera/camera/infra2/image_rect_raw'),
        ('right/camera_info', '/camera/camera/infra2/camera_info')
    ]

    # Convert args_rs to the appropriate format for launch_arguments
    launch_arguments = [(key, value) for key, value in args_rs.items()]

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')]),
            launch_arguments=launch_arguments
        ),

        # Nodes to launch       
        Node(
            package='rtabmap_odom', executable='stereo_odometry', output='screen',
            parameters=[stereo_odometry_params],  # List format
            remappings=remappings
        ),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[rtabmap_params],  # List format
            remappings=remappings,
            arguments=['-d']
        ),

        # Uncomment this if you need to use rtabmap_viz
        # Node(
        #     package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        #     parameters=[rtabmap_params],
        #     remappings=remappings
        # ),
    ])
