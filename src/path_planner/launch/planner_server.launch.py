import os 
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory


def generate_launch_description():
    planner_yaml=os.path.join(get_package_share_directory('path_planner'),'config','planner_server.yaml')
    controller_yaml=os.path.join(get_package_share_directory('path_planner'),'config','nav_controller_server.yaml')
    bt_navigator_yaml=os.path.join(get_package_share_directory('path_planner'),'config','bt_navigator.yaml')
    behavior_yaml=os.path.join(get_package_share_directory('path_planner'),'config',behavior.yaml)
   # recovery_yaml=os.path.join(get_package_share_directory('path_planner'),'config','recovery_server.yaml')

    planner_server_node=Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[planner_yaml]
    )

    nav_controller_node=Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[controller_yaml]
    )

    nav_behavior_node=Node(
        package="nav2_behaviors",
        executable="behavior_server",
        output='screen',
        parameters=[behavior_yaml]
    )

    nav_bt_navigator_node=Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        output='screen',
        parameters=[bt_navigator_yaml]
    )
    
    planner_lifecycle_node=Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_planner',
        output='screen',
        parameters=[{'use_sim_time':True},
                    {'autostart':True},
                    {'node_names':['planner_server','controller_server',
                                   'bt_navigator']}
                     ]
    )


    return LaunchDescription(
        [planner_server_node,
        nav_controller_node,
        nav_bt_navigator_node,
        planner_lifecycle_node])