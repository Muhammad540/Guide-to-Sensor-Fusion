import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    package_name = 'turtlebot3_localization'
    params_file = LaunchConfiguration('params_file')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(get_package_share_directory(package_name), 'params', 'nav2_amcl_params.yaml'),
        description='Full path to the ROS2 parameters file to use for the nav2_amcl node')
    
    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='True',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.',
    )
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )
    
    map_file = os.path.join(get_package_share_directory(
        package_name), "maps", "map.yaml")
    
    rviz_file = os.path.join(get_package_share_directory(
        "map_server"), "params", "rviz_localization_config.rviz")
    
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': True}],
        emulate_tty=True),)
    
    ld.add_action(Node(package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{"use_sim_time": True},
                    {"yaml_filename": map_file}],
        emulate_tty=True),)
    
    ld.add_action(Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file],
        respawn=use_respawn,
        arguments=['--ros-args', '--log-level', log_level],
        respawn_delay=2.0,
        remappings=[('/tf', 'tf'),
                    ('/tf_static', 'tf_static')]
    ))
    
    ld.add_action(Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_mapper",
        output="screen",
        parameters=[{"use_sim_time": True},
                    {"autostart": True},
                    {"node_names": ["map_server","amcl"]}],
        emulate_tty=True))

    return ld