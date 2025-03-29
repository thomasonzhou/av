import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import pathlib
import os

def generate_launch_description():

    pkg_share = pathlib.Path(FindPackageShare(package="av_description").find(
        "av_description"
    ))

    xacro_path = pkg_share / "urdf/av.urdf.xacro"

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_launch_arg = DeclareLaunchArgument(name="use_sim_time", default_value="true")

    robot_state_publisher_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("av_description"),
                        "launch",
                        "description.launch.py",
                    ]
                ),
            ]
        ),
        launch_arguments=dict(use_sim_time=use_sim_time).items(),
    )


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution([
            FindPackageShare(package='gazebo_ros'), 'launch', 'gazebo.launch.py'])]),
        )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'av'],
                        output='screen')

    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                name="model",
                default_value=str(xacro_path),
                description="Absolute path to the robot URDF or xacro file"
            ),
            use_sim_time_launch_arg,
            robot_state_publisher_node,
            gazebo,
            spawn_entity
        ]
    )
