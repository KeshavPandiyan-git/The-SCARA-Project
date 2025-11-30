#!/usr/bin/env python3

"""
Simple script to publish joint states so robot appears in RViz
Run this manually if needed
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time


def main():
    rclpy.init()
    node = Node('simple_joint_publisher')
    
    pub = node.create_publisher(JointState, '/joint_states', 10)
    
    joint_state = JointState()
    joint_state.name = ['joint1', 'joint2', 'joint3', 'joint_z']
    joint_state.position = [0.0, 0.0, 0.0, 0.0]
    
    print("Publishing joint states...")
    
    while rclpy.ok():
        now = node.get_clock().now()
        joint_state.header.stamp = now.to_msg()
        pub.publish(joint_state)
        time.sleep(0.1)
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()

