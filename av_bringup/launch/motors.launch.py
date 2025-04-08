import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

TEST_MOTOR_ID   = 0x7D
LEFT_MOTOR_ID   = 0x7E
RIGHT_MOTOR_ID  = 0x7F

def generate_launch_description():
    nodes = []

    # our odometry uses the position encoder for state updates
    override_joint_limits = [{'anguler_position.max': 10e100}, {'anguler_position.min': -10e100}]

    nodes.append(Node(
        package='cybergear_socketcan_driver',
        executable='cybergear_velocity_driver_node',
        name='cybergear_right',
        parameters=[{'device_id': RIGHT_MOTOR_ID}, {'joint_name': 'right_wheel_joint'}, *override_joint_limits]
    ))

    nodes.append(Node(
        package='cybergear_socketcan_driver',
        executable='cybergear_velocity_driver_node',
        name='cybergear_left',  
        parameters=[{'device_id': LEFT_MOTOR_ID}, {'joint_name': 'left_wheel_joint'}, *override_joint_limits]
    ))

    socketcan_bridge_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros2_socketcan'), 'launch/socket_can_bridge.launch.xml')
        ),
        launch_arguments={'interface': 'can0'}.items()
    )

    return LaunchDescription([
        *nodes,
        socketcan_bridge_launch
    ])
