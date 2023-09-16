import os
import time
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    # Create the LaunchDescription object
    ld = LaunchDescription()
    map_package_name="ros2_mapping"
    nav_package_name="navigator_planner"
    
    

    # Specify the paths to the launch files
    
    #path to map_server_launch file.
    map_server_launch_file=os.path.join(get_package_share_directory(map_package_name),"launch","nav2_map_server.launch.py")
    map_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(map_server_launch_file)
    )
    
    #path to mapping_launch file.
    mapping_launch_file=os.path.join(get_package_share_directory(map_package_name),"launch","online_async_launch.py")
    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mapping_launch_file)
    )
    
      #path to navigation_launch file.
    nav_launch_file=os.path.join(get_package_share_directory(nav_package_name),"launch","navigation.launch.py")
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav_launch_file)
    )
    
    #define launch arguments.
    launch_file_arg=LaunchConfiguration('launch_file')
    launch_file=DeclareLaunchArgument('launch_file',default_value="map")
    
    if launch_file_arg == 'mapping':
        ld.add_action(mapping_launch)
    else:
         ld.add_action(map_server_launch)
 
    # Add the launch actions to the LaunchDescription
    #ld.add_action(laser_launch)
    ld.add_action(launch_file)
    ld.add_action(TimerAction(period=90.0,actions=[nav_launch]))

    return ld
