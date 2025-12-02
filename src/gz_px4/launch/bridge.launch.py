"""
ROS 2 Launch File for launching the ros_gz_bridge publishing:
- Simulation Time @ /clock
- Lidar Pointcloud @ /lidar_front/points_raw
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory("gz_px4")
    
    config_file = os.path.join(pkg_share, "config", "bridge_config.yaml")
    
    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_ros_bridge",
        output="screen",
        parameters=[{'config_file': config_file},
                    {'use_sim_time': True}],
    )

    return LaunchDescription([bridge_node])


