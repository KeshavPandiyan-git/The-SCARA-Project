"""
Configuration file for SCARA Robot GUI
"""
# Robot physical parameters
L1 = 228.0  # mm - Link 1 length
L2 = 136.5  # mm - Link 2 length

# Joint limits (degrees)
J1_MIN = -90
J1_MAX = 266
J2_MIN = -150
J2_MAX = 150
J3_MIN = -162
J3_MAX = 162
Z_MIN = 0
Z_MAX = 150

# Stepper motor conversion factors
THETA1_ANGLE_TO_STEPS = 44.444444
THETA2_ANGLE_TO_STEPS = 35.555555
PHI_ANGLE_TO_STEPS = 10.0
Z_DISTANCE_TO_STEPS = 100.0

# Serial communication
BAUD_RATE = 115200
SERIAL_TIMEOUT = 1.0

# Gripper settings
GRIPPER_MIN = 0
GRIPPER_MAX = 100
GRIPPER_OFFSET = 50  # Processing code adds 50

# Motor settings
SPEED_MIN = 500
SPEED_MAX = 4000
ACCELERATION_MIN = 500
ACCELERATION_MAX = 4000



