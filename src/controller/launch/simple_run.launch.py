from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # IMUのlaunchファイルのパスを取得
    bno055_launch_file = os.path.join(
        get_package_share_directory('bno055'),
        'launch',
        'bno055.launch.py'
    )

    return LaunchDescription([
        # キーボード操作
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop',
            output='screen',
            prefix='xterm -e',
        ),

        # ジョイコン操作
        # Node(
        #     package='joy',
        #     executable='joy_node',
        #     name='joy',
        # ),
        # Node(
        #     package='controller',
        #     executable='teleop_joy_control',
        #     name='teleop_joy_control',
        # ),
        
        # 各モーターへ目標速度を送信
        Node(
            package='controller',
            executable='rover',
            name='twist_to_rover',
            parameters=[{'tread': 0.5}],
        ),
        Node(
            package='controller',
            executable='wheel_ctl_serial_master',
            name='wheel_ctl_serial_master',
        ),
        Node(
            package='controller',
            executable='odom_calc',
            name='odom_calc',
        ),
        # IMUの取得
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bno055_launch_file)
        ),
    ])
