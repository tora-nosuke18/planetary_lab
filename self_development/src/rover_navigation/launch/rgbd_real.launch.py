import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    rgbd_odom_parameters=[{
        'frame_id': 'base_link',        #VO固有のフレーム
        'odom_frame_id': 'vo',        #カメラの設置位置のオフセットを設定
        'nitial_pose' : '0.35 0.0 0.4 0.0 0.0 0.0',        # a superimportant parameter, when we fuse sensor data with ekf nodes.
        'odom_guess_frame_id': 'odom',
        'publish_null_when_lost': False,
          'subscribe_depth':True,
          'subscribe_odom_info':False,
          'approx_sync':False,
          'wait_imu_to_init':False,
          'publish_tf': True,
        'queue_size': 10,
        ' wait_imu_to_init':True
        }]

    rtabmap_parameters=[{
          'frame_id':'base_link',
        'odom_frame_id': "vo", 
          'subscribe_depth':True,
          'subscribe_odom_info':False,
          'approx_sync':True,
          'wait_imu_to_init':True,
          'GridGlobal/MinSize': '100',
        'wait_for_transform_duration': 0.05,
        'queue_size': 10,
        }]

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

    rgbd_odom_remappings=[
          ('odom', '/vo'),
          ('imu', '/imu/refiltered'),
          ('rgb/image', '/camera/color/image_raw'),
          ('rgb/camera_info', '/camera/color/camera_info'),
          ('depth/image', '/camera/aligned_depth_to_color/image_raw')]

    rtabmap_remappings=[
          ('odom', '/vo'),
          ('imu', '/imu/data'),
          ('rgb/image', '/camera/color/image_raw'),
          ('rgb/camera_info', '/camera/color/camera_info'),
          ('depth/image', '/camera/aligned_depth_to_color/image_raw')]

    nav2_params = LaunchConfiguration(
        'params', default=[os.path.join(
                get_package_share_directory('rover_navigation'), 'params'),
                           '/nav2_real.yaml']
    )

    ekf_params = LaunchConfiguration(
        'params', default=[os.path.join(
                get_package_share_directory('rover_navigation'), 'params'),
                           '/ekf_real.yaml']
    )

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'unite_imu_method', default_value='1',
            description='0-None, 1-copy, 2-linear_interpolation. Use unite_imu_method:="1" if imu topics stop being published.'),

        # Make sure IR emitter is enabled
        SetParameter(name='depth_module.emitter_enabled', value=1),

        # Launch camera driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('realsense2_camera'), 'launch'),
                '/rs_launch.py']),
                launch_arguments={'camera_namespace': '',
                                  'enable_gyro': 'true',
                                  'enable_accel': 'true',
                                  'unite_imu_method': LaunchConfiguration('unite_imu_method'),
                                  'align_depth.enable': 'true',
                                  'enable_sync': 'true',
                                  'rgb_camera.profile': '640x360x30',
                                  'accel_qos': 'DEFAULT',
                                  'gyro_qos': 'DEFAULT'}.items(),
        ),

        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=rgbd_odom_parameters,
            remappings=rgbd_odom_remappings),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=rtabmap_parameters,
            remappings=rtabmap_remappings,
            arguments=['-d']),

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


        # Compute quaternion of the IMU
        Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
            parameters=[{'use_mag': False, 
                         'world_frame':'enu', 
                         'publish_tf':False,
                         'gain':1.0}],
            remappings=[('imu/data_raw', '/camera/imu')],
            name='first_imu_filter'),

        # Node(
        #     package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
        #     parameters=[{'use_mag': False, 
        #                  'world_frame':'enu', 
        #                  'publish_tf':False,
        #                  'gain':1.0}],
        #     remappings=[('imu/data_raw', '/imu/filtered'),
        #                 ('/imu/data', '/imu/refiltered')],
        #     name='second_imu_filter'),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([os.path.join(
        #         get_package_share_directory('nav2_bringup'), 'launch'),
        #         '/navigation_launch.py']),
        #     launch_arguments ={'params_file' :[nav2_params]}.items(),        # we must set use_sim_time to True
        # ),
    
        # Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='ekf_filter_node',
        #     output='screen',
        #     parameters=[ekf_params],
        #    ),

        # Node(
        #     package='rover_navigation',
        #     executable='wz_filter_node',
        #     name='wz_filter_node',
        #     output='screen',
        # ),

        # Node(
        #     package='rover_navigation',
        #     executable='imu_filter_node',
        #     name='imu_filter_node',
        #     output='screen',
        # )
    ])