from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # パッケージの共有ディレクトリの取得
    pkg_share = get_package_share_directory('rover_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # ワールドファイルのパス
    world_file_path = 'world/display.world'
    world_path = os.path.join(pkg_share, world_file_path)

    # Gazeboの起動
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )
    
    return LaunchDescription([gzserver_cmd, gzclient_cmd])