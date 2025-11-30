"""
Serial communication with Arduino
"""
import serial
import serial.tools.list_ports
import config
from PyQt5.QtCore import QObject, pyqtSignal, QThread
import time


class SerialWorker(QThread):
    """Worker thread for serial communication"""
    data_received = pyqtSignal(str)
    error_occurred = pyqtSignal(str)
    
    def __init__(self, port, baud_rate):
        super().__init__()
        self.port = port
        self.baud_rate = baud_rate
        self.serial_conn = None
        self.running = False
    
    def run(self):
        """Run serial communication in background thread"""
        try:
            self.serial_conn = serial.Serial(
                self.port,
                self.baud_rate,
                timeout=config.SERIAL_TIMEOUT
            )
            time.sleep(2)  # Wait for Arduino to reset
            self.running = True
            
            while self.running:
                if self.serial_conn and self.serial_conn.in_waiting:
                    data = self.serial_conn.readline().decode('utf-8').strip()
                    if data:
                        self.data_received.emit(data)
                time.sleep(0.01)
        except Exception as e:
            self.error_occurred.emit(str(e))
        finally:
            if self.serial_conn:
                self.serial_conn.close()
    
    def stop(self):
        """Stop the serial communication"""
        self.running = False
        if self.serial_conn:
            self.serial_conn.close()
    
    def write_data(self, data):
        """Write data to serial port"""
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.write(data.encode('utf-8'))
            except Exception as e:
                self.error_occurred.emit(f"Write error: {str(e)}")


class SerialManager(QObject):
    """Manages serial communication with Arduino"""
    connection_status = pyqtSignal(bool)
    error_occurred = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self.worker = None
        self.connected = False
    
    @staticmethod
    def get_available_ports():
        """Get list of available serial ports"""
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]
    
    def connect(self, port):
        """Connect to Arduino"""
        try:
            if self.worker and self.worker.isRunning():
                self.disconnect()
            
            self.worker = SerialWorker(port, config.BAUD_RATE)
            self.worker.error_occurred.connect(self.error_occurred.emit)
            self.worker.start()
            self.connected = True
            self.connection_status.emit(True)
        except Exception as e:
            self.error_occurred.emit(f"Connection error: {str(e)}")
            self.connected = False
            self.connection_status.emit(False)
    
    def disconnect(self):
        """Disconnect from Arduino"""
        if self.worker:
            self.worker.stop()
            self.worker.wait()
            self.worker = None
        self.connected = False
        self.connection_status.emit(False)
    
    def send_command(self, save_status, run_status, j1, j2, j3, z, gripper, speed, acceleration):
        """Send command to Arduino in expected format"""
        if not self.connected or not self.worker:
            return False
        
        # Format: saveStatus,runStatus,theta1,theta2,phi,z,gripper,speed,acceleration
        data = f"{save_status},{run_status},{j1},{j2},{j3},{z},{gripper},{speed},{acceleration}\n"
        self.worker.write_data(data)
        return True



