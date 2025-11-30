# SCARA Robot ROS2 Project

ROS2 integration for SCARA Robot with RViz visualization and kinematics control.

**⚠️ Status: Work in Progress**

The ROS2 integration is currently under development. The robot model (URDF) and topics are configured, but RViz visualization may have display issues that are still being debugged.

## Project Structure

```
scara_robot_ros2/
├── src/
│   ├── scara_description/        # Robot URDF model
│   │   ├── urdf/
│   │   │   ├── scara_robot.urdf      # URDF model
│   │   │   └── scara_robot.urdf.xacro # Xacro version
│   │   ├── launch/
│   │   │   └── display_urdf.launch.py  # Launch RViz visualization
│   │   ├── rviz/
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   └── scara_control/            # Control and kinematics
│       ├── scara_control/
│       │   ├── kinematics.py     # Forward/Inverse kinematics
│       │   └── __init__.py
│       ├── launch/
│       ├── resource/
│       ├── setup.py
│       └── package.xml
│
└── README.md
```

## Robot Specifications

- **Link 1 (L1)**: 228mm (0.228m)
- **Link 2 (L2)**: 136.5mm (0.1365m)
- **Joints**:
  - J1 (Base rotation): -90° to 266°
  - J2 (Shoulder/Elbow): -150° to 150°
  - J3 (Wrist rotation - Phi): -162° to 162°
  - Z-axis (Prismatic): 0 to 150mm (0 to 0.15m)

## Prerequisites

### Install Docker and Docker Compose

This project uses Docker for a consistent ROS2 environment across all platforms.

**Windows:**
- Install [Docker Desktop](https://www.docker.com/products/docker-desktop)

**Ubuntu:**
```bash
sudo apt update
sudo apt install docker.io docker-compose
sudo usermod -aG docker $USER
# Log out and back in, or run: newgrp docker
```

**Arch Linux:**
```bash
sudo pacman -S docker docker-compose
sudo systemctl enable docker
sudo systemctl start docker
sudo usermod -aG docker $USER
# Activate group: newgrp docker
```

### Required ROS2 Packages

```bash
# Core packages (usually installed with ROS2)
ros2-common-interfaces
ros2-urdf
ros2-robot-state-publisher
ros2-joint-state-publisher
ros2-joint-state-publisher-gui
ros2-rviz2

# Python packages
pip install numpy
```

## Installation

### Step 1: Start Docker Container

```bash
cd scara_robot_ros2
docker-compose run --rm ros2
```

This will start a ROS2 Humble container with the workspace mounted.

### Step 2: Build the Workspace (Inside Container)

```bash
cd /workspace
colcon build
source install/setup.bash
```

### Step 3: Enable X11 Forwarding (For RViz)

**On host machine (before starting container):**
```bash
# Ubuntu/Arch Linux
xhost +local:docker
```

**Note**: Install `xhost` if not available:
- Ubuntu: `sudo apt install xorg-xhost`
- Arch: `sudo pacman -S xorg-xhost`

## Usage

### Launch RViz Visualization

```bash
# Make sure ROS2 is sourced
source /opt/ros/humble/setup.bash  # Adjust for your ROS2 version
source install/setup.bash

# Launch visualization
ros2 launch scara_description display_urdf.launch.py
```

This will:
- Start robot_state_publisher (publishes robot description)
- Start joint_state_publisher (provides joint states)
- Start joint_state_publisher_gui (GUI to control joints)
- Launch RViz2 for visualization

### Control Robot Joints

When the GUI is running:
1. Use **Joint State Publisher GUI** to control joints manually
2. Move sliders to see robot move in RViz
3. Adjust J1, J2, J3, and Z-axis positions

### View Robot Model

In RViz:
1. Add displays:
   - **RobotModel**: Shows robot visualization
   - **TF**: Shows coordinate frames
   - **Joint States**: Shows joint information
2. Use mouse to rotate/zoom/pan the view

## Troubleshooting

### "Package not found" error

Make sure you've:
1. Built the workspace: `colcon build`
2. Sourced the workspace: `source install/setup.bash`
3. Sourced ROS2: `source /opt/ros/humble/setup.bash`

### URDF errors

Check URDF syntax:
```bash
check_urdf src/scara_description/urdf/scara_robot.urdf
```

### RViz not showing robot

1. Check if robot_state_publisher is running: `ros2 node list`
2. Check if robot description is published: `ros2 topic echo /robot_description`
3. Add RobotModel display in RViz manually

### Joint State Publisher GUI not appearing

Install it:
```bash
sudo apt install ros-humble-joint-state-publisher-gui
# Or for your ROS2 distribution
```

## Next Steps

1. **Add kinematics nodes**: Create ROS2 nodes for forward/inverse kinematics
2. **Gazebo simulation**: Add Gazebo integration for physics simulation
3. **Control integration**: Connect to Arduino for hardware control
4. **MoveIt integration**: Add motion planning capabilities

## Files Overview

### URDF Model (`scara_robot.urdf`)
- Complete robot model with 4 DOF
- Correct link lengths (L1=228mm, L2=136.5mm)
- Joint limits matching hardware specifications
- Visual and collision models

### Launch Files (`display_urdf.launch.py`)
- Automatically starts all required nodes
- Configures RViz for visualization
- Sets up robot_state_publisher and joint_state_publisher

### Kinematics (`kinematics.py`)
- Forward kinematics: Joint angles → End-effector position
- Inverse kinematics: End-effector position → Joint angles
- Workspace checking

## Development

### Rebuild after changes

```bash
cd scara_robot_ros2
colcon build
source install/setup.bash
```

### Check package structure

```bash
ros2 pkg list | grep scara
```

### View robot in RViz

```bash
ros2 launch scara_description display_urdf.launch.py
```

## License

MIT License - Educational Use

## Credits

Based on the SCARA Robot project by Dejan Nedelkovski (HowToMechatronics.com)
ROS2 integration and URDF model created for educational purposes.

