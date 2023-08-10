from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    laser_scan_matcher_node = Node(
        package='laser_scan_matcher',
        executable='laser_scan_matcher',
        output='screen'
    )

    ld.add_action(laser_scan_matcher_node)

    return ld