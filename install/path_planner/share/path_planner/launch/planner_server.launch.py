import os 
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration,PythonExpression
from nav2_common.launch import RewrittenYaml
from ament_index_python import get_package_share_directory


def generate_launch_description():
    planner_yaml=os.path.join(get_package_share_directory('path_planner'),'config','nav_params.yaml')
    controller_yaml=os.path.join(get_package_share_directory('path_planner'),'config','controller_server.yaml')
    bt_navigator_yaml=os.path.join(get_package_share_directory('path_planner'),'config','bt_navigator.yaml')
    behavior_yaml=os.path.join(get_package_share_directory('path_planner'),'config','behavior.yaml')
   # recovery_yaml=os.path.join(get_package_share_directory('path_planner'),'config','recovery_server.yaml')
    params_file=LaunchConfiguration('params_file')
    autostart=LaunchConfiguration('autostart')
    namespace = LaunchConfiguration('namespace')
    default_bt_xml_filename=LaunchConfiguration('default_bt_xml_filename')

    param_substitutions={'autostart':autostart,
                         'default_bt_xml_filename':default_bt_xml_filename}


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
        get_package_share_directory('path_planner'),'config','navigate_to_pose_w_replanning_and_recovery.xml'),
        description='Full path to the behavior tree xml file to use')
    
    planner_server_node=Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params]
    )

    nav_controller_node=Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[controller_yaml]
    )

    nav_behavior_node=Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name='behavior_server',
        output='screen',
        parameters=[behavior_yaml]
    )

    nav_bt_navigator_node=Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
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
                                   'behavior_server','bt_navigator']}])


    return LaunchDescription(
        [
        declare_default_bt_xml_filename,
        declare_autostart_cmd,
        declare_namespace,
        declare_params_file_cmd,
        nav_controller_node,
        planner_server_node,
        nav_behavior_node,
        nav_bt_navigator_node,
        planner_lifecycle_node])