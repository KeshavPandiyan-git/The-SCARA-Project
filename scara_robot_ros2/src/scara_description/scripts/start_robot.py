#!/usr/bin/env python3
"""
Simple script to start everything - robot publisher and joint states
Just run this ONE script!
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from robot_state_publisher import RobotStatePublisher
import time
import sys
import os

# Read URDF file
pkg_share = '/workspace/install/scara_description/share/scara_description'
urdf_file = os.path.join(pkg_share, 'urdf', 'scara_robot.urdf')

if not os.path.exists(urdf_file):
    print(f"ERROR: URDF file not found at {urdf_file}")
    sys.exit(1)

with open(urdf_file, 'r') as f:
    robot_desc = f.read()

rclpy.init()

# Create nodes
robot_pub_node = Node('robot_state_publisher')
joint_pub_node = Node('joint_publisher')

# Robot state publisher
robot_pub = RobotStatePublisher(robot_pub_node)
robot_pub.set_robot_description(robot_desc)

# Joint state publisher
joint_pub = joint_pub_node.create_publisher(JointState, '/joint_states', 10)

print("Starting robot publisher and joint states...")
print("Press Ctrl+C to stop")

try:
    while rclpy.ok():
        # Publish joint states
        js = JointState()
        js.header.stamp = robot_pub_node.get_clock().now().to_msg()
        js.name = ['joint1', 'joint2', 'joint3', 'joint_z']
        js.position = [0.0, 0.0, 0.0, 0.0]
        joint_pub.publish(js)
        
        # Spin both nodes
        rclpy.spin_once(robot_pub_node, timeout_sec=0.1)
        rclpy.spin_once(joint_pub_node, timeout_sec=0.1)
        
        time.sleep(0.1)
except KeyboardInterrupt:
    print("\nStopping...")
finally:
    robot_pub_node.destroy_node()
    joint_pub_node.destroy_node()
    rclpy.shutdown()

