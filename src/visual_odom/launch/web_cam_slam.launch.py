import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    pkg_share = get_package_share_directory('rover_description')

    xacro_file = os.path.join(pkg_share,'urdf','rover.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    stereo_odometry_params = {
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'publish_tf': True,
        'queue_size': 20,
        'approx_sync': True,
'        approx_sync_max_interval': 0.01,
        'use_sim_time' : False,
        'wait_for_transform': 0.22,
        
        'Stereo/MaxDisparity': "512.0",
        # 'Stereo/MinDisparity' : "64.0" ,
        'OdomF2M/MaxSize': '1000',
        # 'GFTT/MinDistance': '10',
        # 'Odom/MinInliers' : "50"
    }

    pkg_stereo_image_proc = get_package_share_directory(
        'stereo_image_proc')

    stereo_image_proc_launch = PathJoinSubstitution(
        [pkg_stereo_image_proc, 'launch', 'stereo_image_proc.launch.py'])

    rtabmap_params = {
        'frame_id': 'base_link',
        'subscribe_stereo': True,
        'subscribe_depth': False,
        'approx_sync': True,
        'wait_for_transform_duration': 0.2,
        'map_always_update': True,
        'queue_size': 10,
        'use_sim_time' : False,

        'GridGlobal/MinSize': '100',
        # 'Vis/MinInliers': 12,
        # 'Mem/UseOdomGravity': True,
        # 'Optimizer/GravitySigma': 0.3,
        # 'Vis/MaxDepth': 3.5,
        # 'Vis/FeatureType': 8
    }

    remappings = [
        ('odom', '/visual_odom'),
        ('left/image_rect', '/web_camera/left/image_rect'),
        ('left/camera_info', '/web_camera/left/camera_info'),
        ('right/image_rect', '/web_camera/right/image_rect'),
        ('right/camera_info', '/web_camera/right/camera_info')
    ]


    return LaunchDescription([
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     output='screen',
        #     parameters=[params]
        # ),
            
        Node(
            package='v4l2_camera', executable='v4l2_camera_node',name='left_cam_node',
            namespace='/web_camera/left',
            parameters=[{'video_device': '/dev/left-video0'},
                        {'camera_frame_id': 'stereo_camera_left_frame'}]
            ),

        Node(
            package='v4l2_camera', executable='v4l2_camera_node',name='right_cam_node',
            namespace='/web_camera/right',
            parameters=[{'video_device': '/dev/right-video2'},
                        {'camera_frame_id': 'stereo_camera_right_frame'}]
            ),

        Node(
            package='tf2_ros', name='cam_link_to_left',
            executable='static_transform_publisher',
            arguments = ['--x', '0', '--y', '0.0', '--z', '0.1', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'stereo_camera_left_frame']
        ),

        Node(
            package='tf2_ros', name='cam_link_to_right',
            executable='static_transform_publisher',
            arguments = ['--x', '0', '--y', '-0.08', '--z', '0.1', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'stereo_camera_right_frame']
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([stereo_image_proc_launch]),
            launch_arguments=[
                ('left_namespace', 'web_camera/left'),
                ('right_namespace', 'web_camera/right'),
                ('disparity_range', '256'),
            ]
        ),
    
        Node(
            package='rtabmap_odom', executable='stereo_odometry', 
            parameters=[stereo_odometry_params],  # List format
            remappings=remappings,
            output='screen'
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
