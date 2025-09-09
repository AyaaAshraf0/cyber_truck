import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch_ros.actions import Node, SetRemap

from launch.actions import TimerAction, ExecuteProcess, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

navigation_dir = get_package_share_directory('navigation')

map_yaml_file = os.path.join(navigation_dir,"maps","classroom_map.yaml")

lifecycle_nodes = ['map_server','amcl' ]

nav2_bringup_dir = get_package_share_directory('nav2_bringup')
navigation_launch_file = os.path.join(nav2_bringup_dir,'launch','navigation_launch.py')
navigation_params = os.path.join(navigation_dir,'config','navigation.yaml')

def generate_launch_description():
    
    map_server = Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[{"yaml_filename":map_yaml_file,
                             "use_sim_time":True}],
                remappings=[("/clock","/fast_clock")],
    )
    
    amcl_node = Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[{"base_frame_id":"base_link",
                             "odom_frame_id":"odom",
                             "scan_topic":"scan",
                             }],
                remappings=[("/clock","/fast_clock")],
    )
    
    lifecycle_manager = Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{'use_sim_time': True},
                            {'autostart': True},
                            {'node_names': lifecycle_nodes}])
    
    deactivate_map_server = TimerAction(
        period=4.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/map_server', 'deactivate'],
            output='screen'
        )]
    )
    
    activate_map_server = TimerAction(
        period=6.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/map_server', 'activate'],
            output='screen'
        )]
    )
    
    navigation_launch = GroupAction(
            actions=[
                SetRemap(src='/clock', dst='/fast_clock'),IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                navigation_launch_file
            ),
            launch_arguments={
                'params_file': navigation_params,
                'use_sim_time': 'true',
                
            }.items(),
            
        )])
    
    ld= LaunchDescription()
    ld.add_action(map_server)
    ld.add_action(amcl_node)
    ld.add_action(lifecycle_manager)
    ld.add_action(deactivate_map_server)
    ld.add_action(activate_map_server)
    ld.add_action(navigation_launch)
    return ld