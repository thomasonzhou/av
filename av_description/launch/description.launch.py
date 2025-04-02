import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import pathlib

def generate_launch_description():

    pkg_share = pathlib.Path(FindPackageShare(package="av_description").find(
        "av_description"
    ))

    xacro_path = pkg_share / "urdf/av.urdf.xacro"

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_launch_arg = DeclareLaunchArgument(name="use_sim_time", default_value="false")
    use_ros2_control = LaunchConfiguration("use_ros2_control")
    use_ros2_control_launch_arg = DeclareLaunchArgument(name="use_ros2_control", default_value="true")
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": ParameterValue(
                    Command(["xacro ", LaunchConfiguration("model"), ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])
                ),
                "use_sim_time": use_sim_time
            }
        ]
    )

    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                name="model",
                default_value=str(xacro_path),
                description="Absolute path to the robot URDF or xacro file"
            ),
            use_sim_time_launch_arg,
            use_ros2_control_launch_arg,
            robot_state_publisher_node
        ]
    )