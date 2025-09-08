import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import  ExecuteProcess

def generate_launch_description():
    gazebo_pkg = FindPackageShare(package="gazebo_ros").find("gazebo_ros")
    cyber_Robot_pkg = FindPackageShare(package="cyber_robt").find("cyber_robt")
    urdf_model_path = '/home/ahmed/ws_aya/install/cyber_robt/share/cyber_robt/urdf/cyber_robt.urdf.xacro'

    # Load URDF contents as string
    # with open(urdf_model_path, 'r') as infp:
    #     robot_desc = infp.read()

    robot_desc = Command([
        'xacro', ' ', 
        urdf_model_path,' ', 
        'use_ignition:=false'])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_pkg,"launch","gazebo.launch.py"))
    )
            
    tf_footprint_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_footprint_base",
        arguments=["0","0","0","0","0","0","base_footprint","base_link"]
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }],
        remappings=[('/imu_plugin/out','/imu')]
    )

    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time':True}]
    )

    # Optional GUI for joints
    joint_state_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time':True}]
    )

    # Spawn robot in Gazebo
    spawn = Node(
        package='gazebo_ros',
        executable="spawn_entity.py",
        arguments=["-topic", 'robot_description', "-entity", "cyber_robt"],
        output="screen",
        parameters=[{'use_sim_time':True}]
    )

   
    
        
    ld = LaunchDescription()
    ld.add_action(gazebo)
    # ld.add_action(tf_footprint_base)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    # ld.add_action(joint_state_gui)
    ld.add_action(spawn)

    return ld