from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 速度指令から各モーターへ目標速度を送信
        Node(
            package="rover_controller",
            executable="rover",
            name="twist_to_rover",
            parameters=[{"tread": 0.5}],
        ),

        # オドメトリの計算
        Node(
            package="rover_controller",
            executable="odom_calc",
            name="odom_calc",
        ),

        # 以下はコメントアウトされたノードのPython版コメント

        # キーボード操作用ノード（コメントアウト）
        # Node(
        #     package="teleop_twist_keyboard",
        #     executable="teleop_twist_keyboard",
        #     name="teleop",
        #     output="screen",
        #     prefix="xterm -e"
        # ),

        # ジョイスティック操作用ノード（コメントアウト）
        # Node(
        #     package="joy",
        #     executable="joy_node",
        #     name="joy"
        # ),
        # Node(
        #     package="controller",
        #     executable="teleop_joy_control",
        #     name="teleop_joy_control"
        # ),

        # シリアル通信ノード（コメントアウト）
        # Node(
        #     package="controller",
        #     executable="wheel_ctl_serial_master",
        #     name="wheel_ctl_serial_master"
        # ),

        # IMU取得ノードのインクルード（コメントアウト）
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         [FindPackageShare("bno055"), '/launch/bno055.launch.py']
        #     )
        # ),
    ])
