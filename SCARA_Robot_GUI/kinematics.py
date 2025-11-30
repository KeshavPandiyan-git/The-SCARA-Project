"""
Forward and Inverse Kinematics for SCARA Robot
"""
import numpy as np
import config


def forward_kinematics(theta1, theta2):
    """
    Calculate end-effector position from joint angles
    
    Args:
        theta1: Joint 1 angle in degrees
        theta2: Joint 2 angle in degrees
    
    Returns:
        x, y: End-effector position in mm
    """
    # Convert to radians
    theta1_rad = np.deg2rad(theta1)
    theta2_rad = np.deg2rad(theta2)
    
    # Forward kinematics equations
    x = config.L1 * np.cos(theta1_rad) + config.L2 * np.cos(theta1_rad + theta2_rad)
    y = config.L1 * np.sin(theta1_rad) + config.L2 * np.sin(theta1_rad + theta2_rad)
    
    return round(x), round(y)


def inverse_kinematics(x, y):
    """
    Calculate joint angles from end-effector position
    
    Args:
        x: End-effector X position in mm
        y: End-effector Y position in mm
    
    Returns:
        theta1, theta2, phi: Joint angles in degrees
        Returns None if position is unreachable
    """
    # Check if position is reachable
    distance = np.sqrt(x**2 + y**2)
    max_reach = config.L1 + config.L2
    min_reach = abs(config.L1 - config.L2)
    
    if distance > max_reach or distance < min_reach:
        return None, None, None
    
    # Calculate theta2 using cosine law
    cos_theta2 = (x**2 + y**2 - config.L1**2 - config.L2**2) / (2 * config.L1 * config.L2)
    
    # Clamp to valid range for acos
    cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)
    
    theta2 = np.arccos(cos_theta2)
    
    # Choose elbow-up or elbow-down configuration (using elbow-up)
    theta2 = -theta2  # Negative for elbow-up configuration
    
    # Calculate theta1
    if y != 0:
        theta1 = np.arctan2(y, x) - np.arctan2(config.L2 * np.sin(theta2), 
                                                 config.L1 + config.L2 * np.cos(theta2))
    else:
        if x > 0:
            theta1 = 0
        else:
            theta1 = np.pi
    
    # Convert to degrees
    theta1_deg = np.rad2deg(theta1)
    theta2_deg = np.rad2deg(theta2)
    
    # Adjust for quadrant
    if x >= 0 and y >= 0:  # 1st quadrant
        theta1_deg = 90 - theta1_deg
    elif x < 0 and y > 0:  # 2nd quadrant
        theta1_deg = 90 - theta1_deg
    elif x < 0 and y < 0:  # 3rd quadrant
        theta1_deg = 270 - theta1_deg
    elif x > 0 and y < 0:  # 4th quadrant
        theta1_deg = -90 - theta1_deg
    elif x < 0 and y == 0:
        theta1_deg = 270 + theta1_deg
    
    # Calculate phi (wrist rotation) to keep gripper parallel to X-axis
    phi = 90 + theta1_deg + theta2_deg
    phi = -phi
    
    # Adjust phi for 3rd quadrant
    if x < 0 and y < 0:
        phi = 270 - theta1_deg - theta2_deg
    
    # Keep phi in valid range
    if abs(phi) > 165:
        phi = 180 + phi
    
    return round(theta1_deg), round(theta2_deg), round(phi)


def is_reachable(x, y):
    """Check if position (x, y) is within robot workspace"""
    distance = np.sqrt(x**2 + y**2)
    max_reach = config.L1 + config.L2
    min_reach = abs(config.L1 - config.L2)
    return min_reach <= distance <= max_reach



