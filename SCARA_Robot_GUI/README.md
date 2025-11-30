# SCARA Robot Control GUI - PyQt5

A modern, cross-platform GUI for controlling a SCARA robot using PyQt5. Developed by Keshav Pandiyan as a replacement for the original Processing-based GUI.

## Features

- **Forward Kinematics**: Control joints directly with sliders
- **Inverse Kinematics**: Input X, Y, Z coordinates to calculate joint angles
- **Real-time Visualization**: 2D top-view visualization of robot position
- **Serial Communication**: Connect to Arduino for hardware control
- **Program Mode**: Save and replay sequences of positions
- **Jog Control**: Fine-tune positions with JOG+/- buttons
- **Workspace Visualization**: See robot reachable area

## Installation

### Prerequisites

- Python 3.7 or higher
- Arduino with SCARA_Robot.ino firmware uploaded
- Serial port access (may require permissions on Linux)

### Install Dependencies

#### Windows

```cmd
pip install -r requirements.txt
```

#### Ubuntu Linux

**Option 1: Using apt (recommended)**
```bash
sudo apt update
sudo apt install python3 python3-pip python3-pyqt5 python3-pyqtgraph python3-serial
```

**Option 2: Using pip**
```bash
pip3 install -r requirements.txt
```

#### Arch Linux

**Option 1: Using pacman (recommended)**
```bash
sudo pacman -S python python-pyqt5 python-pyqtgraph python-pyserial python-numpy
```

**Option 2: Using pip**
```bash
pip install -r requirements.txt
```

## Usage

1. **Upload Arduino Firmware**: Upload `../SCARA_Robot/SCARA_Robot.ino` to your Arduino

2. **Connect Arduino**: Connect Arduino to USB port

3. **Run GUI**:
```bash
cd SCARA_Robot_GUI
python scara_gui.py
```

4. **Connect to Arduino**:
   - Select your serial port from dropdown (usually `/dev/ttyACM0` or `/dev/ttyUSB0`)
   - Click "Refresh" if port not listed
   - Click "Connect"

5. **Control Robot**:
   - Use Forward Kinematics sliders to move joints directly
   - Or use Inverse Kinematics to input X, Y, Z coordinates
   - Save positions and create programs
   - Adjust speed and acceleration as needed

## Configuration

Edit `config.py` to modify:
- Robot link lengths (L1, L2)
- Joint limits
- Stepper motor conversion factors
- Serial port settings

## Troubleshooting

### Serial Port Issues

**Windows:**
- Check Device Manager for COM port number
- Make sure no other program is using the port (close Serial Monitor)

**Linux - Serial port not found:**
- Check if Arduino is connected: `ls /dev/tty*`
- Ubuntu: Add user to dialout group:
  ```bash
  sudo usermod -a -G dialout $USER
  newgrp dialout  # Or log out and back in
  ```
- Arch Linux: Add user to uucp group:
  ```bash
  sudo usermod -a -G uucp $USER
  newgrp uucp  # Or log out and back in
  ```

**Permission denied:**
- Don't run with sudo (not recommended)
- Add user to appropriate group (see above)

**GUI won't start:**
- Check all dependencies are installed: `pip list`
- Check Python version: `python --version`
- Try: `python3 scara_gui.py`

**Import errors:**
- Make sure all Python files are in the same directory
- Check if all required packages are installed: `pip list | grep -i pyqt`

## Project Structure

```
SCARA_Robot_GUI/
├── scara_gui.py          # Main GUI application
├── kinematics.py         # Forward/inverse kinematics calculations
├── serial_comm.py        # Arduino serial communication
├── robot_visualizer.py   # 2D robot visualization
├── config.py             # Configuration settings
├── requirements.txt      # Python dependencies
└── README.md             # This file
```

## Credits

**Original Design**: Based on the SCARA robot project by Dejan Nedelkovski from [HowToMechatronics.com](https://www.howtomechatronics.com/)

**PyQt5 GUI**: Developed by Keshav Pandiyan as a modern, cross-platform replacement for the original Processing-based GUI, with enhanced features and better Linux compatibility.

## License

This project is open source and available for educational purposes.



