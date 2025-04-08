from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    slam_share_dir = FindPackageShare("slam_toolbox").find("slam_toolbox")
    slam_toolbox_launch_path = str(slam_share_dir) + "/launch/online_async_launch.py"

    av_nav2_share_dir = FindPackageShare("av_nav2").find("av_nav2")
    mapper_params_path = str(av_nav2_share_dir) + "/config/mapper_params_online_async.yaml"


    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_toolbox_launch_path),
            launch_arguments={"use_sim_time": "false", 
                                "slam_params_file": str(mapper_params_path)}.items()
        )
    ])
