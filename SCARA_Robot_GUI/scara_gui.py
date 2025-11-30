"""
SCARA Robot Control GUI using PyQt5
Main application file
"""
import sys
import json
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QSlider, QPushButton, QLineEdit,
                             QComboBox, QTextEdit, QFileDialog, QMessageBox,
                             QGroupBox, QGridLayout, QSpinBox)
from PyQt5.QtCore import Qt, pyqtSlot
from PyQt5 import QtCore
import kinematics
import serial_comm
import robot_visualizer
import config


class SCARAGUI(QMainWindow):
    """Main SCARA Robot Control GUI"""
    
    def __init__(self):
        super().__init__()
        self.serial_manager = serial_comm.SerialManager()
        self.positions_list = []  # Store saved positions
        self.active_ik = False  # Flag to prevent IK feedback loop
        
        self.init_ui()
        self.setup_connections()
        
        # Update available ports
        self.update_ports()
    
    def init_ui(self):
        """Initialize the user interface"""
        self.setWindowTitle("SCARA Robot Control - PyQt5")
        self.setGeometry(100, 100, 1400, 900)
        
        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Main layout
        main_layout = QHBoxLayout()
        central_widget.setLayout(main_layout)
        
        # Left panel - Controls
        left_panel = self.create_control_panel()
        main_layout.addWidget(left_panel, 1)
        
        # Right panel - Visualization
        right_panel = self.create_visualization_panel()
        main_layout.addWidget(right_panel, 1)
        
        # Status bar
        self.statusBar().showMessage("Ready")
    
    def create_control_panel(self):
        """Create left control panel"""
        panel = QWidget()
        layout = QVBoxLayout()
        panel.setLayout(layout)
        
        # Title
        title = QLabel("SCARA Robot Control")
        title.setStyleSheet("font-size: 24px; font-weight: bold; padding: 10px;")
        layout.addWidget(title)
        
        # Serial connection group
        serial_group = QGroupBox("Serial Connection")
        serial_layout = QGridLayout()
        
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(200)
        refresh_btn = QPushButton("Refresh")
        refresh_btn.clicked.connect(self.update_ports)
        connect_btn = QPushButton("Connect")
        connect_btn.clicked.connect(self.connect_serial)
        disconnect_btn = QPushButton("Disconnect")
        disconnect_btn.clicked.connect(self.disconnect_serial)
        
        self.connection_status = QLabel("Disconnected")
        self.connection_status.setStyleSheet("color: red; font-weight: bold;")
        
        serial_layout.addWidget(QLabel("Port:"), 0, 0)
        serial_layout.addWidget(self.port_combo, 0, 1)
        serial_layout.addWidget(refresh_btn, 0, 2)
        serial_layout.addWidget(connect_btn, 1, 1)
        serial_layout.addWidget(disconnect_btn, 1, 2)
        serial_layout.addWidget(self.connection_status, 2, 0, 1, 3)
        serial_group.setLayout(serial_layout)
        layout.addWidget(serial_group)
        
        # Forward Kinematics Group
        fk_group = QGroupBox("Forward Kinematics")
        fk_layout = QVBoxLayout()
        
        # Joint 1
        j1_layout = QHBoxLayout()
        j1_layout.addWidget(QLabel("J1 (θ₁):"))
        self.j1_slider = QSlider(Qt.Horizontal)
        self.j1_slider.setRange(config.J1_MIN, config.J1_MAX)
        self.j1_slider.setValue(0)
        self.j1_slider.valueChanged.connect(self.on_joint_changed)
        self.j1_value = QLabel("0°")
        self.j1_value.setMinimumWidth(50)
        j1_layout.addWidget(self.j1_slider)
        j1_layout.addWidget(self.j1_value)
        
        self.j1_jog_minus = QPushButton("JOG-")
        self.j1_jog_plus = QPushButton("JOG+")
        self.j1_jog_value = QSpinBox()
        self.j1_jog_value.setRange(1, 20)
        self.j1_jog_value.setValue(1)
        self.j1_jog_minus.clicked.connect(lambda: self.jog_joint(1, -1))
        self.j1_jog_plus.clicked.connect(lambda: self.jog_joint(1, 1))
        
        j1_jog_layout = QHBoxLayout()
        j1_jog_layout.addWidget(self.j1_jog_minus)
        j1_jog_layout.addWidget(self.j1_jog_value)
        j1_jog_layout.addWidget(self.j1_jog_plus)
        
        fk_layout.addLayout(j1_layout)
        fk_layout.addLayout(j1_jog_layout)
        
        # Joint 2
        j2_layout = QHBoxLayout()
        j2_layout.addWidget(QLabel("J2 (θ₂):"))
        self.j2_slider = QSlider(Qt.Horizontal)
        self.j2_slider.setRange(config.J2_MIN, config.J2_MAX)
        self.j2_slider.setValue(0)
        self.j2_slider.valueChanged.connect(self.on_joint_changed)
        self.j2_value = QLabel("0°")
        self.j2_value.setMinimumWidth(50)
        j2_layout.addWidget(self.j2_slider)
        j2_layout.addWidget(self.j2_value)
        
        self.j2_jog_minus = QPushButton("JOG-")
        self.j2_jog_plus = QPushButton("JOG+")
        self.j2_jog_value = QSpinBox()
        self.j2_jog_value.setRange(1, 20)
        self.j2_jog_value.setValue(1)
        self.j2_jog_minus.clicked.connect(lambda: self.jog_joint(2, -1))
        self.j2_jog_plus.clicked.connect(lambda: self.jog_joint(2, 1))
        
        j2_jog_layout = QHBoxLayout()
        j2_jog_layout.addWidget(self.j2_jog_minus)
        j2_jog_layout.addWidget(self.j2_jog_value)
        j2_jog_layout.addWidget(self.j2_jog_plus)
        
        fk_layout.addLayout(j2_layout)
        fk_layout.addLayout(j2_jog_layout)
        
        # Joint 3
        j3_layout = QHBoxLayout()
        j3_layout.addWidget(QLabel("J3 (φ):"))
        self.j3_slider = QSlider(Qt.Horizontal)
        self.j3_slider.setRange(config.J3_MIN, config.J3_MAX)
        self.j3_slider.setValue(0)
        self.j3_slider.valueChanged.connect(self.on_joint_changed)
        self.j3_value = QLabel("0°")
        self.j3_value.setMinimumWidth(50)
        j3_layout.addWidget(self.j3_slider)
        j3_layout.addWidget(self.j3_value)
        
        self.j3_jog_minus = QPushButton("JOG-")
        self.j3_jog_plus = QPushButton("JOG+")
        self.j3_jog_value = QSpinBox()
        self.j3_jog_value.setRange(1, 20)
        self.j3_jog_value.setValue(1)
        self.j3_jog_minus.clicked.connect(lambda: self.jog_joint(3, -1))
        self.j3_jog_plus.clicked.connect(lambda: self.jog_joint(3, 1))
        
        j3_jog_layout = QHBoxLayout()
        j3_jog_layout.addWidget(self.j3_jog_minus)
        j3_jog_layout.addWidget(self.j3_jog_value)
        j3_jog_layout.addWidget(self.j3_jog_plus)
        
        fk_layout.addLayout(j3_layout)
        fk_layout.addLayout(j3_jog_layout)
        
        # Z-axis
        z_layout = QHBoxLayout()
        z_layout.addWidget(QLabel("Z:"))
        self.z_slider = QSlider(Qt.Horizontal)
        self.z_slider.setRange(config.Z_MIN, config.Z_MAX)
        self.z_slider.setValue(100)
        self.z_slider.valueChanged.connect(self.on_joint_changed)
        self.z_value = QLabel("100 mm")
        self.z_value.setMinimumWidth(60)
        z_layout.addWidget(self.z_slider)
        z_layout.addWidget(self.z_value)
        
        self.z_jog_minus = QPushButton("JOG-")
        self.z_jog_plus = QPushButton("JOG+")
        self.z_jog_value = QSpinBox()
        self.z_jog_value.setRange(1, 20)
        self.z_jog_value.setValue(1)
        self.z_jog_minus.clicked.connect(lambda: self.jog_joint(4, -1))
        self.z_jog_plus.clicked.connect(lambda: self.jog_joint(4, 1))
        
        z_jog_layout = QHBoxLayout()
        z_jog_layout.addWidget(self.z_jog_minus)
        z_jog_layout.addWidget(self.z_jog_value)
        z_jog_layout.addWidget(self.z_jog_plus)
        
        fk_layout.addLayout(z_layout)
        fk_layout.addLayout(z_jog_layout)
        
        fk_group.setLayout(fk_layout)
        layout.addWidget(fk_group)
        
        # Inverse Kinematics Group
        ik_group = QGroupBox("Inverse Kinematics")
        ik_layout = QGridLayout()
        
        ik_layout.addWidget(QLabel("X (mm):"), 0, 0)
        self.x_input = QLineEdit()
        self.x_input.setText("365")
        self.x_input.editingFinished.connect(self.on_ik_changed)
        ik_layout.addWidget(self.x_input, 0, 1)
        
        ik_layout.addWidget(QLabel("Y (mm):"), 1, 0)
        self.y_input = QLineEdit()
        self.y_input.setText("0")
        self.y_input.editingFinished.connect(self.on_ik_changed)
        ik_layout.addWidget(self.y_input, 1, 1)
        
        ik_layout.addWidget(QLabel("Z (mm):"), 2, 0)
        self.z_input = QLineEdit()
        self.z_input.setText("100")
        self.z_input.editingFinished.connect(self.on_ik_changed)
        ik_layout.addWidget(self.z_input, 2, 1)
        
        move_btn = QPushButton("Move to Position")
        move_btn.clicked.connect(self.move_to_position)
        ik_layout.addWidget(move_btn, 3, 0, 1, 2)
        
        ik_group.setLayout(ik_layout)
        layout.addWidget(ik_group)
        
        # Gripper control
        gripper_group = QGroupBox("Gripper")
        gripper_layout = QVBoxLayout()
        
        gripper_label_layout = QHBoxLayout()
        gripper_label_layout.addWidget(QLabel("CLOSE"))
        gripper_label_layout.addStretch()
        gripper_label_layout.addWidget(QLabel("OPEN"))
        gripper_layout.addLayout(gripper_label_layout)
        
        self.gripper_slider = QSlider(Qt.Horizontal)
        self.gripper_slider.setRange(config.GRIPPER_MIN, config.GRIPPER_MAX)
        self.gripper_slider.setValue(100)
        self.gripper_slider.valueChanged.connect(self.on_gripper_changed)
        self.gripper_value = QLabel("100")
        gripper_layout.addWidget(self.gripper_slider)
        gripper_layout.addWidget(self.gripper_value)
        
        gripper_group.setLayout(gripper_layout)
        layout.addWidget(gripper_group)
        
        # Motor settings
        motor_group = QGroupBox("Motor Settings")
        motor_layout = QGridLayout()
        
        motor_layout.addWidget(QLabel("Speed:"), 0, 0)
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(config.SPEED_MIN, config.SPEED_MAX)
        self.speed_slider.setValue(500)
        self.speed_slider.valueChanged.connect(self.on_motor_settings_changed)
        self.speed_value = QLabel("500")
        motor_layout.addWidget(self.speed_slider, 0, 1)
        motor_layout.addWidget(self.speed_value, 0, 2)
        
        motor_layout.addWidget(QLabel("Acceleration:"), 1, 0)
        self.accel_slider = QSlider(Qt.Horizontal)
        self.accel_slider.setRange(config.ACCELERATION_MIN, config.ACCELERATION_MAX)
        self.accel_slider.setValue(500)
        self.accel_slider.valueChanged.connect(self.on_motor_settings_changed)
        self.accel_value = QLabel("500")
        motor_layout.addWidget(self.accel_slider, 1, 1)
        motor_layout.addWidget(self.accel_value, 1, 2)
        
        motor_group.setLayout(motor_layout)
        layout.addWidget(motor_group)
        
        # Program controls
        program_group = QGroupBox("Program Control")
        program_layout = QVBoxLayout()
        
        btn_layout = QHBoxLayout()
        self.save_btn = QPushButton("Save Position")
        self.save_btn.clicked.connect(self.save_position)
        self.run_btn = QPushButton("Run Program")
        self.run_btn.clicked.connect(self.run_program)
        self.run_btn.setStyleSheet("background-color: green; color: white;")
        self.run_status = False
        btn_layout.addWidget(self.save_btn)
        btn_layout.addWidget(self.run_btn)
        program_layout.addLayout(btn_layout)
        
        clear_btn = QPushButton("Clear Program")
        clear_btn.clicked.connect(self.clear_program)
        program_layout.addWidget(clear_btn)
        
        # Position list
        self.position_list = QTextEdit()
        self.position_list.setReadOnly(True)
        self.position_list.setMaximumHeight(100)
        program_layout.addWidget(QLabel("Saved Positions:"))
        program_layout.addWidget(self.position_list)
        
        program_group.setLayout(program_layout)
        layout.addWidget(program_group)
        
        layout.addStretch()
        
        return panel
    
    def create_visualization_panel(self):
        """Create right visualization panel"""
        panel = QWidget()
        layout = QVBoxLayout()
        panel.setLayout(layout)
        
        # Robot visualizer
        self.visualizer = robot_visualizer.RobotVisualizer()
        layout.addWidget(self.visualizer)
        
        # Current position display
        pos_group = QGroupBox("Current Position")
        pos_layout = QGridLayout()
        
        self.x_display = QLabel("X: 365 mm")
        self.y_display = QLabel("Y: 0 mm")
        self.z_display = QLabel("Z: 100 mm")
        
        pos_layout.addWidget(self.x_display, 0, 0)
        pos_layout.addWidget(self.y_display, 0, 1)
        pos_layout.addWidget(self.z_display, 0, 2)
        
        pos_group.setLayout(pos_layout)
        layout.addWidget(pos_group)
        
        return panel
    
    def setup_connections(self):
        """Setup signal connections"""
        self.serial_manager.connection_status.connect(self.on_connection_status)
        self.serial_manager.error_occurred.connect(self.on_serial_error)
    
    def update_ports(self):
        """Update available serial ports"""
        ports = serial_comm.SerialManager.get_available_ports()
        self.port_combo.clear()
        self.port_combo.addItems(ports if ports else ["No ports available"])
    
    @pyqtSlot()
    def connect_serial(self):
        """Connect to selected serial port"""
        port = self.port_combo.currentText()
        if port and port != "No ports available":
            self.serial_manager.connect(port)
            self.statusBar().showMessage(f"Connecting to {port}...")
        else:
            QMessageBox.warning(self, "No Port", "Please select a valid serial port")
    
    @pyqtSlot()
    def disconnect_serial(self):
        """Disconnect from serial port"""
        self.serial_manager.disconnect()
    
    @pyqtSlot(bool)
    def on_connection_status(self, connected):
        """Handle connection status change"""
        if connected:
            self.connection_status.setText("Connected")
            self.connection_status.setStyleSheet("color: green; font-weight: bold;")
            self.statusBar().showMessage("Connected to Arduino")
        else:
            self.connection_status.setText("Disconnected")
            self.connection_status.setStyleSheet("color: red; font-weight: bold;")
            self.statusBar().showMessage("Disconnected")
    
    @pyqtSlot(str)
    def on_serial_error(self, error):
        """Handle serial errors"""
        QMessageBox.critical(self, "Serial Error", f"Error: {error}")
        self.statusBar().showMessage(f"Error: {error}")
    
    def on_joint_changed(self):
        """Handle joint slider changes"""
        if self.active_ik:
            return
        
        j1 = self.j1_slider.value()
        j2 = self.j2_slider.value()
        j3 = self.j3_slider.value()
        z = self.z_slider.value()
        
        # Update labels
        self.j1_value.setText(f"{j1}°")
        self.j2_value.setText(f"{j2}°")
        self.j3_value.setText(f"{j3}°")
        self.z_value.setText(f"{z} mm")
        
        # Calculate forward kinematics
        x, y = kinematics.forward_kinematics(j1, j2)
        
        # Update display
        self.x_display.setText(f"X: {x} mm")
        self.y_display.setText(f"Y: {y} mm")
        self.z_display.setText(f"Z: {z} mm")
        
        # Update inputs
        self.x_input.setText(str(x))
        self.y_input.setText(str(y))
        self.z_input.setText(str(z))
        
        # Update visualization
        self.visualizer.update_robot(j1, j2, j3, z)
        
        # Send to Arduino
        self.send_command()
    
    def on_ik_changed(self):
        """Handle inverse kinematics input changes"""
        try:
            x = float(self.x_input.text())
            y = float(self.y_input.text())
            z = float(self.z_input.text())
            
            # Check if reachable
            if not kinematics.is_reachable(x, y):
                QMessageBox.warning(self, "Unreachable", 
                                  f"Position ({x}, {y}) is outside robot workspace")
                return
            
            # Calculate inverse kinematics
            theta1, theta2, phi = kinematics.inverse_kinematics(x, y)
            
            if theta1 is None:
                QMessageBox.warning(self, "IK Error", "Could not calculate inverse kinematics")
                return
            
            # Update sliders
            self.active_ik = True
            self.j1_slider.setValue(int(theta1))
            self.j2_slider.setValue(int(theta2))
            self.j3_slider.setValue(int(phi))
            self.z_slider.setValue(int(z))
            self.active_ik = False
            
            # Update visualization
            self.visualizer.update_robot(theta1, theta2, phi, z)
            
            # Update display
            self.x_display.setText(f"X: {x} mm")
            self.y_display.setText(f"Y: {y} mm")
            self.z_display.setText(f"Z: {z} mm")
            
        except ValueError:
            QMessageBox.warning(self, "Invalid Input", "Please enter valid numbers")
    
    def move_to_position(self):
        """Move robot to specified IK position"""
        self.on_ik_changed()
        self.send_command()
    
    def on_gripper_changed(self):
        """Handle gripper slider change"""
        value = self.gripper_slider.value()
        self.gripper_value.setText(str(value))
        self.send_command()
    
    def on_motor_settings_changed(self):
        """Handle motor settings change"""
        speed = self.speed_slider.value()
        accel = self.accel_slider.value()
        self.speed_value.setText(str(speed))
        self.accel_value.setText(str(accel))
        self.send_command()
    
    def jog_joint(self, joint, direction):
        """Jog joint by specified amount"""
        jog_value = 0
        if joint == 1:
            jog_value = self.j1_jog_value.value()
            new_value = self.j1_slider.value() + (direction * jog_value)
            self.j1_slider.setValue(max(config.J1_MIN, min(config.J1_MAX, new_value)))
        elif joint == 2:
            jog_value = self.j2_jog_value.value()
            new_value = self.j2_slider.value() + (direction * jog_value)
            self.j2_slider.setValue(max(config.J2_MIN, min(config.J2_MAX, new_value)))
        elif joint == 3:
            jog_value = self.j3_jog_value.value()
            new_value = self.j3_slider.value() + (direction * jog_value)
            self.j3_slider.setValue(max(config.J3_MIN, min(config.J3_MAX, new_value)))
        elif joint == 4:
            jog_value = self.z_jog_value.value()
            new_value = self.z_slider.value() + (direction * jog_value)
            self.z_slider.setValue(max(config.Z_MIN, min(config.Z_MAX, new_value)))
    
    def save_position(self):
        """Save current position to program"""
        j1 = self.j1_slider.value()
        j2 = self.j2_slider.value()
        j3 = self.j3_slider.value()
        z = self.z_slider.value()
        gripper = self.gripper_slider.value()
        
        position = {
            'j1': j1,
            'j2': j2,
            'j3': j3,
            'z': z,
            'gripper': gripper
        }
        
        self.positions_list.append(position)
        pos_text = f"Position {len(self.positions_list)}: J1={j1}°, J2={j2}°, J3={j3}°, Z={z}mm, G={gripper}\n"
        self.position_list.append(pos_text)
        
        # Send save command
        self.send_command(save_status=1)
    
    def run_program(self):
        """Run saved program"""
        if not self.positions_list:
            QMessageBox.warning(self, "No Program", "Please save at least one position first")
            return
        
        if not self.run_status:
            self.run_status = True
            self.run_btn.setText("Stop Program")
            self.run_btn.setStyleSheet("background-color: red; color: white;")
            self.send_command(run_status=1)
        else:
            self.run_status = False
            self.run_btn.setText("Run Program")
            self.run_btn.setStyleSheet("background-color: green; color: white;")
            self.send_command(run_status=0)
    
    def clear_program(self):
        """Clear saved program"""
        reply = QMessageBox.question(self, "Clear Program", 
                                    "Are you sure you want to clear all saved positions?",
                                    QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.positions_list.clear()
            self.position_list.clear()
            self.send_command(save_status=2)  # Clear command for Arduino
    
    def send_command(self, save_status=0, run_status=None):
        """Send command to Arduino"""
        if run_status is None:
            run_status = 1 if self.run_status else 0
        
        j1 = self.j1_slider.value()
        j2 = self.j2_slider.value()
        j3 = self.j3_slider.value()
        z = self.z_slider.value()
        gripper = self.gripper_slider.value() + config.GRIPPER_OFFSET  # Add offset like Processing
        speed = self.speed_slider.value()
        accel = self.accel_slider.value()
        
        self.serial_manager.send_command(save_status, run_status, j1, j2, j3, z, gripper, speed, accel)


def main():
    """Main function"""
    app = QApplication(sys.argv)
    app.setStyle('Fusion')  # Modern look
    
    window = SCARAGUI()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()



