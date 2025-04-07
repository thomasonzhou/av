import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_dir = get_package_share_directory('av_bringup')

    motors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'motors.launch.py')
        )
    )

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'control.launch.py')
        )
    )
    delayed_control_launch = TimerAction(
        period=5.0, 
        actions=[control_launch]
    )

    # sensors
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'lidar.launch.py')
        )
    )

    depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'depth_camera.launch.py')
        )
    )

    return LaunchDescription([
        motors_launch,
        delayed_control_launch,
        lidar_launch,
        depth_camera_launch,
    ])
