#!/bin/bash
echo "Checking if robot_state_publisher is subscribed to joint_states..."
echo ""
echo "Run this in Docker container:"
echo "ros2 node info /robot_state_publisher"
echo ""
echo "Look for '/joint_states' in the subscription list"
