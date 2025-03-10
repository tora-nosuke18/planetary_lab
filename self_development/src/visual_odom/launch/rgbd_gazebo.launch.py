from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default="True")     #実機の場合はFalse、シミュレーションの場合はTrue


    stereo_odometry_params = {
        'frame_id': 'base_link',        #VO固有のフレーム
        'odom_frame_id': 'visual_odom',        #カメラの設置位置のオフセットを設定
        'nitial_pose' : '0.35 0.0 0.4 0.0 0.0 0.0',        # a superimportant parameter, when we fuse sensor data with ekf nodes.
        'odom_guess_frame_id': 'odom',
        'publish_null_when_lost': False,
        'publish_tf': False,        #VOのみの場合はTrue
        'queue_size': 10,
        'approx_sync': False,
        'use_sim_time' : use_sim_time,
        'wait_for_transform': 0.0,
        'OdomF2M/MaxSize': '1000',
        'GFTT/MinDistance': '10',
        'Odom/MinInliers' : "50",
        'Odom/ResetCountdown': "1",
    }

    rtabmap_params = {
        #VOのみの場合はvisual_odomを指定
        'odom_frame_id': "odom", 
        # 'odom_frame_id': "visual_odom",
        'subscribe_stereo': False,
        'subscribe_depth': True,
        'approx_sync': False,
        'wait_for_transform_duration': 0.0,
        'map_always_update': True,
        'queue_size': 10,
        'use_sim_time' : use_sim_time,
        'GridGlobal/MinSize': '100',
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
        'max_obstacles_height': 1.5

    }

    rgbd_odom_remappings = [
        ('odom', '/visual/odom'),
        ('rgb/image', 'color/image_raw'),
        ('rgb/camera_info', 'color/camera_info'),
        ('depth/image', 'depth/depth/image_raw'),
    ]

    rtabmap_remappings = [
        ('odom', '/odometry/filtered'),        #VOのみの場合は/visual/odomを指定,ekf起動時は/odometry/filteredを指定
        ('rgb/image', 'color/image_raw'),
        ('rgb/camera_info', 'color/camera_info'),
        ('depth/image', 'depth/depth/image_raw'),
    ]

    return LaunchDescription([      
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', 
            parameters=[stereo_odometry_params],  # List format
            remappings=rgbd_odom_remappings,
        ),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[rtabmap_params],  # List format
            remappings=rtabmap_remappings, 
            arguments=['--delete_db_on_start'], # This will delete the previous database
        ),

        # Node(
        #     package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        #     parameters=[rtabmap_params],
        #     remappings=remappings),

        # create segmented point cloud
        # while rtabmap_slam provides with octomap_ground & octomap_obstacles, following nodes are able to adjust the point cloud parameters.
        # And obstacles topic is used for voxel and obstacle layer of costmap2d.
        Node(
            package='rtabmap_util', executable='point_cloud_xyz', output='screen',
            parameters=[depth_to_cloud_params],  # List format
            remappings=rtabmap_remappings, 
        ),

        Node(
            package='rtabmap_util', executable='obstacles_detection', output='screen',
            parameters=[obstacles_detection_params],  # List format
            # remappings=remappings, 
        ),

    ])
