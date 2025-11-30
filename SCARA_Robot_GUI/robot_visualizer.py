"""
2D Robot Visualization Widget using PyQtGraph
"""
from PyQt5.QtWidgets import QWidget, QVBoxLayout
from PyQt5.QtCore import Qt
import pyqtgraph as pg
import numpy as np
import config


class RobotVisualizer(QWidget):
    """2D visualization of SCARA robot"""
    
    def __init__(self):
        super().__init__()
        self.init_ui()
        self.joint_angles = [0, 0, 0]  # theta1, theta2, phi
        self.z_position = 100
    
    def init_ui(self):
        """Initialize the UI"""
        layout = QVBoxLayout()
        self.setLayout(layout)
        
        # Create plot widget
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setLabel('left', 'Y (mm)')
        self.plot_widget.setLabel('bottom', 'X (mm)')
        self.plot_widget.setTitle('SCARA Robot - Top View')
        self.plot_widget.setAspectLocked(True)
        self.plot_widget.setRange(xRange=[-400, 400], yRange=[-400, 400])
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        
        layout.addWidget(self.plot_widget)
        
        # Robot visualization line
        self.robot_line = self.plot_widget.plot([], [], pen=pg.mkPen('b', width=3))
        
        # Joint markers
        self.joint_markers = self.plot_widget.plot([], [], pen=None, symbol='o', 
                                                   symbolSize=8, symbolBrush='g')
        
        # End-effector marker
        self.end_effector = self.plot_widget.plot([], [], pen=None, symbol='o', 
                                                   symbolSize=12, symbolBrush='r')
        
        # Workspace circle
        self.draw_workspace()
    
    def draw_workspace(self):
        """Draw robot workspace"""
        max_reach = config.L1 + config.L2
        min_reach = abs(config.L1 - config.L2)
        
        # Outer workspace circle
        angles = np.linspace(0, 2*np.pi, 100)
        x_outer = max_reach * np.cos(angles)
        y_outer = max_reach * np.sin(angles)
        self.plot_widget.plot(x_outer, y_outer, pen=pg.mkPen('g', width=1, style=Qt.DashLine))
        
        # Inner workspace circle (if needed)
        if min_reach > 0:
            x_inner = min_reach * np.cos(angles)
            y_inner = min_reach * np.sin(angles)
            self.plot_widget.plot(x_inner, y_inner, pen=pg.mkPen('g', width=1, style=Qt.DashLine))
    
    def update_robot(self, theta1, theta2, phi, z):
        """Update robot visualization"""
        self.joint_angles = [theta1, theta2, phi]
        self.z_position = z
        
        # Convert to radians
        theta1_rad = np.deg2rad(theta1)
        theta2_rad = np.deg2rad(theta2)
        
        # Calculate joint positions
        x0, y0 = 0, 0  # Base position
        x1 = config.L1 * np.cos(theta1_rad)
        y1 = config.L1 * np.sin(theta1_rad)
        
        x2 = x1 + config.L2 * np.cos(theta1_rad + theta2_rad)
        y2 = y1 + config.L2 * np.sin(theta1_rad + theta2_rad)
        
        # Draw robot links
        x_robot = [x0, x1, x2]
        y_robot = [y0, y1, y2]
        
        self.robot_line.setData(x_robot, y_robot)
        
        # Draw joint markers (base and elbow)
        self.joint_markers.setData([x0, x1], [y0, y1])
        
        # Draw end-effector
        self.end_effector.setData([x2], [y2])
        
        # Update title with current position
        self.plot_widget.setTitle(f'SCARA Robot - X: {x2:.1f}mm, Y: {y2:.1f}mm, Z: {z}mm')



