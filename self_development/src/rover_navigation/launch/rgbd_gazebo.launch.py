import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


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


    nav2_params = LaunchConfiguration(
        'params', default=[os.path.join(
                get_package_share_directory('rover_navigation'), 'params'),
                           '/nav2_real.yaml']
    )

    ekf_params = LaunchConfiguration(
        'params', default=[os.path.join(
                get_package_share_directory('rover_navigation'), 'params'),
                           '/ekf_gazebo.yaml']
    )

    return LaunchDescription([      
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', 
            parameters=[stereo_odometry_params],  
            remappings=rgbd_odom_remappings,
        ),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[rtabmap_params],  
            remappings=rtabmap_remappings, 
            arguments=['--delete_db_on_start'], #ノードを再起動する際、前回のマップデータを削除するオプション
        ),

        # 地面と障害物にセグメント化された点群を作成するノード
        # rtabmap_slamはoctomap_groundとoctomap_obstaclesをPublishするが, 以下のノードは点群に関するパラメータをいじれる.
        # "obstacles"トピックはcostmap2dのvoxelとobstacleレイヤーに読み込ませている.
        Node(
            package='rtabmap_util', executable='point_cloud_xyz', output='screen',
            parameters=[depth_to_cloud_params],  
            remappings=rtabmap_remappings, 
        ),

        Node(
            package='rtabmap_util', executable='obstacles_detection', output='screen',
            parameters=[obstacles_detection_params],  
            # remappings=remappings,
        ),



        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('nav2_bringup'), 'launch'),
                '/navigation_launch.py']),
            launch_arguments ={'params_file' :[nav2_params],'use_sim_time': use_sim_time}.items(),        # we must set use_sim_time to True
        ),
    
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params, {'use_sim_time': use_sim_time}],
           ),

        # Node(
        #     package='rover_navigation',
        #     executable='odom_base_broadcaster',
        #     name='odom_base_broadcaster',
        #     output='screen',
        # ),
    ])
