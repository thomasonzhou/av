import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
import pathlib
import os 
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_share = pathlib.Path(FindPackageShare(package="av_description").find(
        "av_description"
    ))

    xacro_path = pkg_share / "urdf/av.urdf.xacro"

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_launch_arg = DeclareLaunchArgument(name="use_sim_time", default_value="false")

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


    robot_description_content = Command([
    'xacro ', LaunchConfiguration("model")
    ])

    robot_description_param = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    controller_params = PathJoinSubstitution([
        FindPackageShare("av_description"),
        'config',
        'controllers.yaml'
        ])

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_params, robot_description_param],)

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])
    
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont'],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad'],
    )

    delayed_joint_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broadcaster_spawner],
        )
    )

    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                name="model",
                default_value=str(xacro_path),
                description="Absolute path to the robot URDF or xacro file"
            ),
            use_sim_time_launch_arg,
            robot_state_publisher_node,
            delayed_controller_manager,
            delayed_diff_drive_spawner,
            # delayed_joint_broadcaster_spawner
        ]
    )
