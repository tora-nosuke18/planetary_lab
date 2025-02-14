from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    stereo_odometry_params = {
        'frame_id': 'base_link',
        'odom_frame_id': 'visual_odom',
        'publish_tf': True,
        'queue_size': 10,
        'approx_sync': False,
        'use_sim_time' : True,
        # 'wait_for_transform': True,
        'Stereo/MaxDisparity': "256",
        'Stereo/MinDisparity' : "0" ,
        'OdomF2M/MaxSize': '1000',
        'GFTT/MinDistance': '10',
        'Odom/MinInliers' : "50",
        'Odom/ResetCountdown': "1",
    }

    rtabmap_params = {
        'odom_frame_id': "visual_odom",
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

    depth_to_cloud_params = {
        'queue_size': 10,
        'voxel_size ':0.0,
        'decimation': 0,
        'min_depth': 0.0,
        'max_depth': 0.0,

    }

    obstacles_detection_params = {
        'queue_size': 10,
        'frame_id': 'base_link',

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
            parameters=[stereo_odometry_params],  # List format
            remappings=remappings,
        ),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[rtabmap_params],  # List format
            remappings=remappings, 
            arguments=['--delete_db_on_start'], # This will delete the previous database
        ),

<<<<<<< HEAD
        # Node(
        #     package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        #     parameters=[rtabmap_params],
        #     remappings=remappings),
=======

        # create segmented point cloud
        # while rtabmap_slam provides with octomap_ground & octomap_obstacles, following nodes are able to adjust the point cloud parameters.
        # And obstacles topic is used for voxel and obstacle layer of costmap2d.
        Node(
            package='rtabmap_util', executable='point_cloud_xyz', output='screen',
            parameters=[depth_to_cloud_params],  # List format
            remappings=remappings, 
        ),

        Node(
            package='rtabmap_util', executable='obstacles_detection', output='screen',
            parameters=[obstacles_detection_params],  # List format
            # remappings=remappings, 
        ),

>>>>>>> ba00dfa (miscellaneous changes)
    ])
