import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
                'frame_id': 'base_laser',
                'angle_compensate': True,
                'serial_baudrate':115200,
                'scan_mode': 'Sensitivity',
                'scan_frequency':10.0
                
            }]
        )
    ])