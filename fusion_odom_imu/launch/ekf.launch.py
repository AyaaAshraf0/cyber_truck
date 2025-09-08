import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



fusion_package = FindPackageShare(package="fusion_odom_imu").find("fusion_odom_imu")
config_file = os.path.join(fusion_package,"config","ekf_node.yaml")

def generate_launch_description():
    ekf_node = Node(
        package= "robot_localization",
        executable= "ekf_node",
        parameters=[config_file],
        remappings= [('/clock','/fast_clock')]
    )

    ld = LaunchDescription()
    ld.add_action(ekf_node)

    return ld