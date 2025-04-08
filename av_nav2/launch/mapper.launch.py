from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    slam_share_dir = FindPackageShare("slam_toolbox")
    slam_toolbox_launch_path = [slam_share_dir, "/launch/online_async_launch.py"]

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_toolbox_launch_path),
            launch_arguments={"use_sim_time": "false"}.items()
        )
    ])
