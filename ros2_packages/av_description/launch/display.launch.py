import pathlib
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    pkg_share = pathlib.Path(FindPackageShare(package="av_description").find(
        "av_description"
    ))

    xacro_path = pkg_share / "urdf/av.urdf.xacro"
    default_rviz_config_path = pkg_share / "config/robot_description.rviz"

    use_sim_time = LaunchConfiguration("use_sim_time")


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

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        condition=launch.conditions.UnlessCondition(LaunchConfiguration("gui")),
        parameters=[
            {
                "use_sim_time": use_sim_time,
            }
        ],
    )
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=launch.conditions.IfCondition(LaunchConfiguration("gui")),
        parameters=[
            {
                "use_sim_time": use_sim_time,
            }
        ],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
        parameters=[
            {
                "use_sim_time": use_sim_time,
            }
        ],
    )

    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                name="use_sim_time",
                default_value="false",
                description="Flag to enable usage of simulation time",
            ),
            DeclareLaunchArgument(
                name="gui",
                default_value="True",
                description="Flag to enable joint_state_publisher_gui",
            ),
            DeclareLaunchArgument(
                name="model",
                default_value=str(xacro_path),
                description="Absolute path to the robot URDF or xacro file"
            ),
            DeclareLaunchArgument(
                name="rvizconfig",
                default_value=str(default_rviz_config_path),
                description="Absolute path to rviz config file",
            ),
            joint_state_publisher_node,
            joint_state_publisher_gui_node,
            robot_state_publisher_node,
            rviz_node,
        ]
    )