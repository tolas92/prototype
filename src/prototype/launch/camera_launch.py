"""Examine how gscam2 and gscam interact with image_transport."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

  
   uncompressed_node=Node(
            package='image_transport',
            executable='republish',
            output='screen',
            namespace='camera2',
            arguments=['compressed','raw'],
            remappings=[
                ('in/compressed','/camera1/out/compressed'),
                ('out','/camera2/uncompressed')
                ],)

    # GSCam uses image_transport and advertises on these topics:
    #       /namespace/camera/camera_info
    #       /namespace/camera/image_raw
    #       /namespace/camera/image_raw/compressed
    #       ... additional transports may be installed
    #
    # There is no need to use republish.

   return LaunchDescription([uncompressed_node])
