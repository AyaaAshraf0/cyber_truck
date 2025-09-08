import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import  ExecuteProcess
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cyber_Robot_pkg = FindPackageShare(package="cyber_robt").find("cyber_robt")
    urdf_model_path = os.path.join(cyber_Robot_pkg,'urdf','cyber_robt.urdf.xacro')
    world_file = os.path.join(cyber_Robot_pkg,'worlds','classroom.sdf')
    
    robot_desc = Command(["xacro ",urdf_model_path," use_ignition:=","true"])
    # Load URDF contents as string
    # with open(urdf_model_path, 'r') as infp:
    #     robot_desc = infp.read()

     # Start the robot_state_publisher node to publish robot states
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_desc,
                "use_sim_time": True,
                # "frame_prefix": "/cyber_robt/",
            }
        ],
        remappings=[('/clock', '/fast_clock'),]

    )
    
     # Publish joint states if GUI is not enabled
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{"use_sim_time": True}],
        # remappings=[('/clock', '/fast_clock'),]

        
    )
    
    ign_gazebo = ExecuteProcess(
        cmd=["ign","gazebo","-r",world_file],
        output="screen",
    )
    
    spawn_entity = Node(
        package="ros_ign_gazebo",
        executable="create",
        arguments=["-name", "cyber_robt", "-topic", "robot_description",
                   '-x','0',
                   '-y', '-2.05',
                    '-z', '0.5',
                    '-Y', '1.5708'
                    ],  # Robot name in the simulation  # Source topic for robot description  # Spawn position (X)  # Spawn position (Y)  # Spawn position (Z)  # Spawn orientation (Yaw)
        output="screen",
        parameters=[{"use_sim_time": True, 
                     
                     
                     }],
        remappings=[('/clock', '/fast_clock'),]

    )

    bridge= Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        arguments=[
                    # "/world/empty/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
                    "/joint_states@sensor_msgs/msg/JointState@ignition.msgs.Model",
                    "/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry",
                    "/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
                    "/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
                    "/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
                    "/model/cyber_robt/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V",
        ],
        parameters=[{"use_sim_time": True}],
    
        output="screen",
        remappings=[
            ('/model/cyber_robt/tf', '/tf'), ('/lidar','/scan'),
                  ('/clock', '/fast_clock') ]
    )

    lidar_tf_static = Node(
        package= "tf2_ros",
        executable= "static_transform_publisher",
        name= "static_lidar_tf",
        parameters=[{"use_sim_time": True}],
        arguments=["0","0","0","0","0","0","Lidar","cyber_robt/base_link/gpu_lidar"],
        remappings=[('/clock', '/fast_clock'),]
    )

    clock_bridge= Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=[
                    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                      
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
        remappings=[('/clock', '/fast_clock'),]
    ) 
    ld = LaunchDescription()
    # Add launch actions to start Gazebo, the robot state publisher, and RViz
    ld.add_action(ign_gazebo)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(spawn_entity)
    ld.add_action(bridge)
    ld.add_action(clock_bridge)
    ld.add_action(lidar_tf_static)
    # ld.add_action(rviz_launch)
    
    return ld