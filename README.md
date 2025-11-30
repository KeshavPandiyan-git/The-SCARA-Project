# The SCARA Project

A complete SCARA (Selective Compliance Assembly Robot Arm) robot project featuring Arduino control, PyQt5 GUI, and ROS2 integration. This project includes hardware design files, firmware, control software, and simulation capabilities.

## 📋 Project Overview

This SCARA robot project was originally adapted from an open-source design and has been enhanced with:
- **Arduino-based control system** for real-time motor control
- **PyQt5 GUI** for intuitive robot control and visualization
- **ROS2 integration** for advanced robotics workflows (Work in Progress)
- **RViz visualization** for 3D robot simulation (Work in Progress)

The robot features 4 degrees of freedom:
- **J1 (Base rotation)**: -90° to 266°
- **J2 (Shoulder/Elbow)**: -150° to 150°
- **J3 (Wrist rotation)**: -162° to 162°
- **Z-axis (Prismatic)**: 0 to 150mm

**Robot Specifications:**
- Link 1 (L1): 228mm
- Link 2 (L2): 136.5mm
- Maximum reach: ~364.5mm
- Control accuracy: <1cm placement error

## 🏗️ Project Structure

```
Project SCARA/
├── SCARA_Robot/                    # Arduino firmware
│   ├── SCARA_Robot.ino            # Main firmware (with homing)
│   └── SCARA_Robot_TestMode.ino   # Test mode (no homing)
│
├── SCARA_Robot_GUI/               # PyQt5 GUI application
│   ├── scara_gui.py              # Main GUI application
│   ├── kinematics.py             # Forward/inverse kinematics
│   ├── serial_comm.py            # Arduino serial communication
│   ├── robot_visualizer.py       # 2D robot visualization
│   ├── config.py                 # Configuration settings
│   ├── requirements.txt          # Python dependencies
│   └── README.md                 # GUI-specific documentation
│
├── GUI_for_SCARA_Robot/           # Original Processing GUI (legacy)
│   └── GUI_for_SCARA_Robot.pde   # Processing-based GUI
│
├── scara_robot_ros2/              # ROS2 integration (Work in Progress)
│   ├── src/
│   │   ├── scara_description/    # URDF robot model
│   │   └── scara_control/        # ROS2 control nodes
│   ├── docker-compose.yml        # Docker setup for ROS2
│   └── README.md                 # ROS2-specific documentation
│
└── STL Files/                     # 3D printing files
    ├── Base.STL
    ├── Arm 1.STL, Arm 2.STL
    ├── Gripper components
    └── ... (all mechanical parts)
```

## 🎯 Features

### PyQt5 GUI (Developed by Keshav Pandiyan)
- **Forward Kinematics Control**: Direct joint angle control with sliders
- **Inverse Kinematics Control**: Input X, Y, Z coordinates to calculate joint angles
- **Real-time 2D Visualization**: Top-view visualization of robot position
- **Serial Communication**: Direct Arduino control via USB
- **Program Mode**: Save and replay sequences of positions
- **Jog Control**: Fine-tune positions with incremental buttons
- **Workspace Visualization**: Visual representation of robot reachable area
- **Cross-platform**: Works on Windows, Linux (Ubuntu, Arch Linux)

### Arduino Firmware
- Stepper motor control using AccelStepper library
- Servo control for gripper
- Limit switch-based homing
- Serial communication protocol
- Real-time position feedback

### ROS2 Integration (Work in Progress)
- URDF robot model
- RViz visualization
- Joint state publishing
- TF transforms
- Docker-based setup for easy deployment

## 🚀 Quick Start

### Prerequisites
- Arduino UNO or compatible board
- Python 3.7+ (for GUI)
- Stepper motors and drivers (for physical robot)
- 3D printer (for mechanical parts) - Optional

### 1. Upload Arduino Firmware

1. Open `SCARA_Robot/SCARA_Robot.ino` in Arduino IDE
2. Install required libraries:
   - **AccelStepper**: Available in Arduino Library Manager
   - **Servo**: Built-in Arduino library
3. Select your board and port
4. Upload the sketch

**Note**: If you don't have limit switches connected, use `SCARA_Robot_TestMode.ino` instead.

### 2. Install and Run PyQt5 GUI

#### Windows

1. **Install Python** (if not already installed):
   - Download from [python.org](https://www.python.org/downloads/)
   - Make sure to check "Add Python to PATH" during installation

2. **Install dependencies**:
   ```cmd
   cd SCARA_Robot_GUI
   pip install -r requirements.txt
   ```

3. **Run the GUI**:
   ```cmd
   python scara_gui.py
   ```

4. **Connect to Arduino**:
   - Connect Arduino via USB
   - In GUI, select COM port (e.g., COM3, COM4)
   - Click "Connect"

#### Ubuntu Linux

1. **Install Python and dependencies**:
   ```bash
   sudo apt update
   sudo apt install python3 python3-pip python3-pyqt5 python3-pyqtgraph python3-serial
   ```

2. **Or use pip**:
   ```bash
   cd SCARA_Robot_GUI
   pip3 install -r requirements.txt
   ```

3. **Fix serial port permissions** (if needed):
   ```bash
   sudo usermod -a -G dialout $USER
   # Log out and back in, or run: newgrp dialout
   ```

4. **Run the GUI**:
   ```bash
   python3 scara_gui.py
   ```

5. **Connect to Arduino**:
   - Select port (usually `/dev/ttyACM0` or `/dev/ttyUSB0`)
   - Click "Connect"

#### Arch Linux

1. **Install dependencies using pacman** (recommended):
   ```bash
   sudo pacman -S python python-pyqt5 python-pyqtgraph python-pyserial python-numpy
   ```

2. **Or use pip**:
   ```bash
   cd SCARA_Robot_GUI
   pip install -r requirements.txt
   ```

3. **Fix serial port permissions**:
   ```bash
   sudo usermod -a -G uucp $USER
   # Activate group: newgrp uucp
   # Or log out and back in
   ```

4. **Run the GUI**:
   ```bash
   python scara_gui.py
   ```

5. **Connect to Arduino**:
   - Select port (usually `/dev/ttyACM0`)
   - Click "Connect"

### 3. Using the GUI

1. **Forward Kinematics Mode**:
   - Use sliders to control J1, J2, J3, and Z-axis directly
   - Watch the 2D visualization update in real-time

2. **Inverse Kinematics Mode**:
   - Enter X, Y, Z coordinates
   - Click "Calculate IK" to compute joint angles
   - Click "Move to Position" to execute

3. **Program Mode**:
   - Move robot to desired positions
   - Click "Save Position" to add to program
   - Click "Run Program" to execute saved sequence

4. **Jog Control**:
   - Use JOG+ and JOG- buttons for fine adjustments
   - Adjust speed and acceleration as needed

## 🔧 Configuration

Edit `SCARA_Robot_GUI/config.py` to modify:
- Robot link lengths (L1, L2)
- Joint limits
- Stepper motor conversion factors
- Serial port settings
- Gripper settings
- Motor speed and acceleration limits

## 🐳 ROS2 Setup (Work in Progress)

The ROS2 integration is currently under development. For instructions on setting up ROS2 with Docker, see `scara_robot_ros2/README.md`.

**Note**: RViz visualization is still being debugged. The robot model and topics are configured, but display issues may occur.

## 🛠️ Hardware Requirements

### Electronics
- Arduino UNO or compatible
- 4x Stepper motors (NEMA 17 recommended)
- 4x Stepper motor drivers (A4988 or DRV8825)
- 1x Servo motor (for gripper)
- 4x Limit switches (for homing)
- Power supply (12V recommended for steppers)

### Mechanical Parts
All STL files are provided in the `STL Files/` directory. You'll need:
- 3D printer (or access to one)
- GT2 timing belts and pulleys
- Smooth rods and bearings
- Screws and fasteners (M3, M4 recommended)

## 📚 Documentation

- **GUI Documentation**: See `SCARA_Robot_GUI/README.md`
- **ROS2 Documentation**: See `scara_robot_ros2/README.md`
- **Arduino Code**: Well-commented in `SCARA_Robot/SCARA_Robot.ino`

## 🐛 Troubleshooting

### Serial Port Issues

**Windows**:
- Check Device Manager for COM port number
- Make sure no other program is using the port (close Serial Monitor)

**Linux**:
- Check available ports: `ls /dev/tty*`
- Add user to appropriate group:
  - Ubuntu: `sudo usermod -a -G dialout $USER`
  - Arch: `sudo usermod -a -G uucp $USER`
- Activate group: `newgrp dialout` (Ubuntu) or `newgrp uucp` (Arch)

### GUI Not Starting

- Check Python version: `python --version` (should be 3.7+)
- Verify all dependencies: `pip list | grep -i pyqt`
- Try running with `python3` instead of `python`

### Arduino Upload Issues

- Close Serial Monitor before uploading
- Try manual reset during upload
- Check board and port settings in Arduino IDE
- Unplug and replug Arduino

### Robot Not Moving

- Check serial connection status in GUI
- Verify motor connections
- Check power supply
- Ensure limit switches are connected (or use TestMode firmware)

## 📝 Credits

### Design Files
All mechanical design files (STL files) and the original Arduino/Processing code are based on the open-source SCARA robot project by **Dejan Nedelkovski** from [HowToMechatronics.com](https://www.howtomechatronics.com/).

**Original Project**: [HowToMechatronics SCARA Robot](https://www.howtomechatronics.com/)

### PyQt5 GUI
The PyQt5 GUI application was developed by **Keshav Pandiyan** as a modern, cross-platform replacement for the original Processing-based GUI.

### ROS2 Integration
ROS2 integration and URDF model created for educational purposes and advanced robotics workflows.

## 📄 License

MIT License

Copyright (c) 2025 Keshav Pandiyan

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

## 📧 Contact

- **GitHub**: [KeshavPandiyan-git](https://github.com/KeshavPandiyan-git)
- **Email**: keshavpandiyan.work@gmail.com, ex24378@gmail.com
- **Repository**: [The-SCARA-Project](https://github.com/KeshavPandiyan-git/The-SCARA-Project)

## 🙏 Acknowledgments

Special thanks to:
- **Dejan Nedelkovski** and **HowToMechatronics.com** for the original open-source SCARA robot design
- The open-source robotics community for tools and libraries

---

**Note**: This project is for educational purposes. The ROS2/RViz integration is currently a work in progress and may have display issues. The PyQt5 GUI and Arduino firmware are fully functional.

