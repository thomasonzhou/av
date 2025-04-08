from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    nav2_share_dir = FindPackageShare("nav2_bringup").find("nav2_bringup")
    nav2_bringup_launch_path = str(nav2_share_dir) + "/launch/navigation_launch.py"

    av_nav2_share_dir = FindPackageShare("av_nav2").find("av_nav2")
    mapper_params_path = str(av_nav2_share_dir) + "/config/nav2_params.yaml"


    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_bringup_launch_path),
            launch_arguments={"use_sim_time": "false", 
                                "params_file": str(mapper_params_path)}.items()
        )
    ])
