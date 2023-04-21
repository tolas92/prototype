import os 
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration,PythonExpression
from nav2_common.launch import RewrittenYaml
from ament_index_python import get_package_share_directory


def generate_launch_description():
    planner_yaml=os.path.join(get_package_share_directory('navigator_planner'),'config','nav_params.yaml')
    controller_yaml=os.path.join(get_package_share_directory('navigator_planner'),'config','controller_server.yaml')
    bt_navigator_yaml=os.path.join(get_package_share_directory('navigator_planner'),'config','bt_navigator.yaml')
    behavior_yaml=os.path.join(get_package_share_directory('navigator_planner'),'config','behavior.yaml')
   # recovery_yaml=os.path.join(get_package_share_directory('path_planner'),'config','recovery_server.yaml')
    params_file=LaunchConfiguration('params_file')
    autostart=LaunchConfiguration('autostart')
    namespace = LaunchConfiguration('namespace')
    default_bt_xml_filename=LaunchConfiguration('default_bt_xml_filename')
    log_level=LaunchConfiguration('log_level')

    param_substitutions={'autostart':autostart,
                         'default_bt_xml_filename':default_bt_xml_filename}

    remappings=[('/tf','tf'),('/tf_static','tf_static')]
    
    configured_params=RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)
    
    declare_namespace=DeclareLaunchArgument('namespace',
                                            default_value='',
                                            description='Top-level namespace')
    

    declare_params_file_cmd=DeclareLaunchArgument(
        'params_file',
        default_value=planner_yaml,
        description='Full path to the ROS2 parameters file to use for planner_server')
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')
    
    declare_default_bt_xml_filename=DeclareLaunchArgument(
        'default_bt_xml_filename',default_value=os.path.join(
        get_package_share_directory('navigator_planner'),'config','navigate_to_pose_w_replanning_and_recovery.xml'),
        description='Full path to the behavior tree xml file to use')
    
    declare_log_level=DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='log level'
    )
    
    nav_planner_server_node=Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params],
        arguments=['--ros-args','--log-level',log_level],
        remappings=remappings
    )

    nav_controller_server_node=Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[controller_yaml],
        arguments=['--ros-args','--log-level',log_level],
        
        remappings=remappings+[('cmd_vel','cmd_vel_nav')]
    )
    nav_smoother_server_node=Node(
        package="nav2_smoother",
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level',log_level],
        remappings=remappings
    )

    nav_behavior_server_node=Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name='behavior_server',
        output='screen',
        parameters=[behavior_yaml],
        arguments=['--ros-args', '--log-level',log_level],
        remappings=remappings
    )

    nav_bt_navigator_node=Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[bt_navigator_yaml]
    )

    nav_waypoint_follower_node=Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level',log_level],
        remappings=remappings
    )

    nav_velocity_smoother_node=Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings +
        [('cmd_vel','cmd_vel_nav'), ('cmd_vel_smoothed','cmd_vel')]
    )
    
    navigator_planner_lifecycle_node=Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_planner',
        output='screen',
        parameters=[{'use_sim_time':True},
                    {'autostart':True},
                    {'node_names':['planner_server','controller_server','smoother_server',
                                   'behavior_server','bt_navigator','waypoint_follower','velocity_smoother']}])


    return LaunchDescription(
        [
        declare_default_bt_xml_filename,
        declare_autostart_cmd,
        declare_namespace,
        declare_params_file_cmd,
        declare_log_level,
        nav_controller_server_node,
        nav_smoother_server_node,
        nav_planner_server_node,
        nav_behavior_server_node,
        nav_bt_navigator_node,
        nav_waypoint_follower_node,
        nav_velocity_smoother_node,
        navigator_planner_lifecycle_node])