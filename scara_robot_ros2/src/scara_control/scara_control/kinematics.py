"""
Forward and Inverse Kinematics for SCARA Robot
"""
import numpy as np

# Robot physical parameters (in meters)
L1 = 0.228   # Link 1 length: 228mm
L2 = 0.1365  # Link 2 length: 136.5mm

# Joint limits (degrees)
J1_MIN = -90
J1_MAX = 266
J2_MIN = -150
J2_MAX = 150
J3_MIN = -162
J3_MAX = 162
Z_MIN = 0.0
Z_MAX = 0.15  # 150mm


def forward_kinematics(theta1, theta2, z=0.0):
    """
    Calculate end-effector position from joint angles
    
    Args:
        theta1: Joint 1 angle in radians
        theta2: Joint 2 angle in radians
        z: Z-axis position in meters
    
    Returns:
        x, y, z: End-effector position in meters
    """
    # Forward kinematics equations
    x = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
    y = L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2)
    
    return x, y, z


def inverse_kinematics(x, y, z=0.0):
    """
    Calculate joint angles from end-effector position
    
    Args:
        x: End-effector X position in meters
        y: End-effector Y position in meters
        z: Z-axis position in meters
    
    Returns:
        theta1, theta2, phi: Joint angles in radians
        Returns None if position is unreachable
    """
    # Check if position is reachable
    distance = np.sqrt(x**2 + y**2)
    max_reach = L1 + L2
    min_reach = abs(L1 - L2)
    
    if distance > max_reach or distance < min_reach:
        return None, None, None
    
    # Calculate theta2 using cosine law
    cos_theta2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    
    # Clamp to valid range for acos
    cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)
    
    theta2 = np.arccos(cos_theta2)
    theta2 = -theta2  # Elbow-up configuration
    
    # Calculate theta1
    if y != 0:
        theta1 = np.arctan2(y, x) - np.arctan2(L2 * np.sin(theta2), 
                                                 L1 + L2 * np.cos(theta2))
    else:
        if x > 0:
            theta1 = 0
        else:
            theta1 = np.pi
    
    # Calculate phi (wrist rotation) to keep gripper orientation
    phi = -(theta1 + theta2)  # Simplified - adjust based on requirements
    
    return theta1, theta2, phi


def is_reachable(x, y):
    """Check if position (x, y) is within robot workspace"""
    distance = np.sqrt(x**2 + y**2)
    max_reach = L1 + L2
    min_reach = abs(L1 - L2)
    return min_reach <= distance <= max_reach

