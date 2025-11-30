#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
import os


def generate_launch_description():
    # Get package share directory
    pkg_share = FindPackageShare('scara_description').find('scara_description')
    
    # URDF file path
    urdf_file = os.path.join(pkg_share, 'urdf', 'scara_robot.urdf')
    
    # Read URDF file
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_desc},
            {'publish_frequency': 50.0},
            {'use_sim_time': False}
        ]
    )
    
    # Publish joint states using ros2 topic pub in a loop
    # This is a workaround since the Python script might not be installed
    joint_state_pub_cmd = """while true; do ros2 topic pub --once /joint_states sensor_msgs/msg/JointState '{name: ["joint1", "joint2", "joint3", "joint_z"], position: [0.0, 0.0, 0.0, 0.0], velocity: [0.0, 0.0, 0.0, 0.0], effort: [0.0, 0.0, 0.0, 0.0]}' 2>/dev/null; sleep 0.1; done"""
    
    joint_state_publisher = ExecuteProcess(
        cmd=['bash', '-c', joint_state_pub_cmd],
        output='screen'
    )
    
    # RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )
    
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        rviz,
    ])
