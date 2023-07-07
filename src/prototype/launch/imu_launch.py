from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    imu_node = Node(
        package='imu_node',
        executable='imu_node',
        output='screen'
    )

    ld.add_action(imu_node)

    return ld