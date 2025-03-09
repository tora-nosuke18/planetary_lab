from launch import LaunchDescription
from launch_ros.actions import Node

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

    rtabmap_params = {
        # 'odom_frame_id': "visual_odom",
        'subscribe_stereo': False,
        'subscribe_depth': True,
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
        ('odom', '/visual/odom'),
        ('rgb/image', 'color/image_raw'),
        ('rgb/camera_info', 'color/camera_info'),
        ('depth/image', 'depth/depth/image_raw'),
    ]

    return LaunchDescription([      
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', 
            parameters=[stereo_odometry_params,
                        {'odom_frame_id': "visual_odom"}],  # List format
            remappings=remappings,
        ),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[rtabmap_params,
                        {'odom_frame_id': "visual_odom"}],  # List format
            remappings=remappings,  # This will delete the previous database
            arguments=['--delete_db_on_start'],
        ),

        # Node(
        #     package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        #     parameters=[rtabmap_params],
        #     remappings=remappings),
    ])
