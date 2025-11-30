#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
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
            {'use_sim_time': False}
        ]
    )
    
    # Python node to publish joint states manually
    joint_state_publisher_node = Node(
        package='scara_description',
        executable='simple_joint_publisher.py',
        name='simple_joint_publisher',
        output='screen'
    )
    
    # RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )
    
    return LaunchDescription([
        robot_state_publisher,
        rviz,
    ])

