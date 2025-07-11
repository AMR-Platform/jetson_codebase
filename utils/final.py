#!/usr/bin/env python3

import sys
import socket
import threading
import time
import numpy as np
import cv2
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QWidget, 
    QLabel, QTextEdit, QPushButton, QGroupBox, QGridLayout,
    QComboBox, QLineEdit, QSplitter, QTabWidget, QCheckBox
)
from PyQt5.QtCore import QThread, pyqtSignal, QTimer, Qt
from PyQt5.QtGui import QPixmap, QImage, QFont
import pyqtgraph as pg
from collections import deque
import select
import termios
import tty
import atexit
import json

class KeyboardController(QThread):
    """Handle keyboard input for teleop control"""
    key_pressed = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self.running = True
        # Save terminal settings
        try:
            self.orig_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
            atexit.register(self.restore_terminal)
        except:
            self.orig_settings = None
    
    def restore_terminal(self):
        if self.orig_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.orig_settings)
    
    def run(self):
        while self.running:
            try:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    ch = sys.stdin.read(1).lower()
                    self.key_pressed.emit(ch)
            except:
                break
    
    def stop(self):
        self.running = False
        self.restore_terminal()

class UDPHandler(QThread):
    """Handle UDP communication with robot"""
    sensor_data = pyqtSignal(dict)
    debug_data = pyqtSignal(dict)
    lidar_data = pyqtSignal(dict)
    echo_data = pyqtSignal(dict)
    velocity_profile_data = pyqtSignal(dict)
    raw_message = pyqtSignal(str)
    
    def __init__(self, listen_ip="192.168.8.157", listen_port=9001, 
                 robot_ip="192.168.8.143", robot_port=9000):
        super().__init__()
        self.listen_ip = listen_ip
        self.listen_port = listen_port
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        self.running = True
        
        # LiDAR chunk assembly
        self.lidar_chunks = {}  # chunk_id -> points
        self.expected_chunks = 0
        
        # Create UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.listen_ip, self.listen_port))
        self.sock.settimeout(1.0)
    
    def send_teleop(self, f, b, l, r):
        """Send teleop command to robot"""
        msg = f"TELEOP,{int(f)},{int(b)},{int(l)},{int(r)}"
        try:
            self.sock.sendto(msg.encode('utf-8'), (self.robot_ip, self.robot_port))
            self.raw_message.emit(f"[SENT] {msg}")
        except Exception as e:
            self.raw_message.emit(f"[ERROR] Failed to send: {e}")
    
    def send_command(self, cmd_dict):
        """Send command packet to robot"""
        # Format: CMD,<mode>,<dbg>,<distance>,<angle>,<maxVel>,<maxOmega>,<lastVel>,<lastOmega>,<linAcc>,<angAcc>
        msg = f"CMD,{cmd_dict.get('mode', 0)},{cmd_dict.get('debug', 0)}," \
              f"{cmd_dict.get('distance', 0.0)},{cmd_dict.get('angle', 0.0)}," \
              f"{cmd_dict.get('maxVel', 0.0)},{cmd_dict.get('maxOmega', 0.0)}," \
              f"{cmd_dict.get('lastVel', 0.0)},{cmd_dict.get('lastOmega', 0.0)}," \
              f"{cmd_dict.get('linAcc', 0.0)},{cmd_dict.get('angAcc', 0.0)}"
        try:
            self.sock.sendto(msg.encode('utf-8'), (self.robot_ip, self.robot_port))
            self.raw_message.emit(f"[SENT] {msg}")
        except Exception as e:
            self.raw_message.emit(f"[ERROR] Failed to send command: {e}")
    
    def parse_sensor_data(self, data):
        """Parse SENSOR packet"""
        try:
            parts = data.split(',')[1:]  # Skip "SENSOR"
            return {
                'yaw': float(parts[0]),
                'roll': float(parts[1]),
                'pitch': float(parts[2]),
                'gyroX': float(parts[3]),
                'gyroY': float(parts[4]),
                'gyroZ': float(parts[5]),
                'accelX': float(parts[6]),
                'accelY': float(parts[7]),
                'accelZ': float(parts[8]),
                'encL': float(parts[9]),
                'encR': float(parts[10]),
                'vbat1': float(parts[11]),
                'vbat2': float(parts[12]),
                'cliffL': float(parts[13]),
                'cliffC': float(parts[14]),
                'cliffR': float(parts[15]),
                'emergency': int(parts[16]),
                'profileDone': int(parts[17])
            }
        except Exception as e:
            self.raw_message.emit(f"[ERROR] Failed to parse SENSOR: {e}")
            return None
    
    def handle_lidar_chunk(self, lidar_dict):
        """Assemble LiDAR chunks into complete scans"""
        chunk_id = lidar_dict['chunk_id']
        total_chunks = lidar_dict['total_chunks']
        points = lidar_dict['points']
        
        # Initialize if this is the first chunk of a new scan
        if chunk_id == 0:
            self.lidar_chunks = {}
            self.expected_chunks = total_chunks
        
        # Store this chunk
        self.lidar_chunks[chunk_id] = points
        
        # Check if we have all chunks
        if len(self.lidar_chunks) == self.expected_chunks:
            # Assemble complete scan
            complete_scan = []
            for i in range(self.expected_chunks):
                if i in self.lidar_chunks:
                    complete_scan.extend(self.lidar_chunks[i])
            
            # Emit the complete scan
            self.lidar_data.emit({'points': complete_scan})
            
            # Clear chunks for next scan
            self.lidar_chunks = {}
    
    def parse_debug_data(self, data):
        """Parse DEBUG packet"""
        try:
            parts = data.split(',')[1:]  # Skip "DEBUG"
            return {
                'spdL': float(parts[0]),
                'spdR': float(parts[1]),
                'vel': float(parts[2]),
                'omg': float(parts[3]),
                'dist': float(parts[4]),
                'ang': float(parts[5]),
                'loopDt': float(parts[6])
            }
        except Exception as e:
            self.raw_message.emit(f"[ERROR] Failed to parse DEBUG: {e}")
            return None
    
    def parse_echo_data(self, data):
        """Parse ECHO packet"""
        try:
            parts = data.split(',')[1:]  # Skip "ECHO"
            return {
                'distance': float(parts[0]),
                'angle': float(parts[1]),
                'maxVel': float(parts[2]),
                'maxOmega': float(parts[3]),
                'lastVel': float(parts[4]),
                'lastOmega': float(parts[5]),
                'linAcc': float(parts[6]),
                'angAcc': float(parts[7])
            }
        except Exception as e:
            self.raw_message.emit(f"[ERROR] Failed to parse ECHO: {e}")
            return None
    
    def parse_lidar_data(self, data):
        """Parse LIDAR packet"""
        try:
            parts = data.split(',')[1:]  # Skip "LIDAR"
            chunk_id = int(parts[0])
            total_chunks = int(parts[1])
            num_points = int(parts[2])
            
            points = []
            for i in range(num_points):
                base_idx = 3 + i * 3
                if base_idx + 2 < len(parts):
                    point = {
                        'azimuth': float(parts[base_idx]),
                        'distance': float(parts[base_idx + 1]),
                        'rssi': int(parts[base_idx + 2])
                    }
                    points.append(point)
            
            return {
                'chunk_id': chunk_id,
                'total_chunks': total_chunks,
                'points': points
            }
        except Exception as e:
            self.raw_message.emit(f"[ERROR] Failed to parse LIDAR: {e}")
            return None
    
    def parse_velocity_profile_data(self, data):
        """Parse VELOCITY_PROFILE packet"""
        try:
            parts = data.split(',')[1:]  # Skip "VELOCITY_PROFILE"
            return {
                'actual_vel': float(parts[0]),
                'actual_omega': float(parts[1]),
                'expected_vel': float(parts[2]),
                'expected_omega': float(parts[3]),
                'spdL': float(parts[4]),
                'spdR': float(parts[5]),
                'target_distance': float(parts[6]),
                'target_angle': float(parts[7]),
                'current_distance': float(parts[8]),
                'current_angle': float(parts[9]),
                'loop_dt': float(parts[10])
            }
        except Exception as e:
            self.raw_message.emit(f"[ERROR] Failed to parse VELOCITY_PROFILE: {e}")
            return None
    
    def run(self):
        while self.running:
            try:
                data, addr = self.sock.recvfrom(4096)
                text = data.decode('utf-8', errors='replace').strip()
                
                # Emit raw message for logging
                self.raw_message.emit(f"[{addr[0]}:{addr[1]}] {text}")
                
                # Parse different message types
                if text.startswith("SENSOR,"):
                    sensor_dict = self.parse_sensor_data(text)
                    if sensor_dict:
                        self.sensor_data.emit(sensor_dict)
                
                elif text.startswith("DEBUG,"):
                    debug_dict = self.parse_debug_data(text)
                    if debug_dict:
                        self.debug_data.emit(debug_dict)
                
                elif text.startswith("ECHO,"):
                    echo_dict = self.parse_echo_data(text)
                    if echo_dict:
                        self.echo_data.emit(echo_dict)
                
                elif text.startswith("LIDAR,"):
                    lidar_dict = self.parse_lidar_data(text)
                    if lidar_dict:
                        self.handle_lidar_chunk(lidar_dict)
                
                elif text.startswith("VELOCITY_PROFILE,"):
                    velocity_dict = self.parse_velocity_profile_data(text)
                    if velocity_dict:
                        self.velocity_profile_data.emit(velocity_dict)
                
                # TODO: Handle MAP and PC (point cloud) packets when implemented
                
            except socket.timeout:
                continue
            except Exception as e:
                self.raw_message.emit(f"[ERROR] UDP receive error: {e}")
    
    def stop(self):
        self.running = False
        self.sock.close()

class LidarVisualizer(QWidget):
    """OpenCV-based LiDAR visualization widget"""
    
    def __init__(self):
        super().__init__()
        self.setMinimumSize(400, 400)
        self.image = np.zeros((400, 400, 3), dtype=np.uint8)
        self.range_max = 10.0  # 10 meters max range
        
    def update_lidar_data(self, points):
        """Update with new LiDAR points"""
        self.image.fill(0)  # Clear image
        
        if not points:
            self.update()
            return
        
        canvas_size = 400
        center = canvas_size // 2
        pixels_per_meter = center / self.range_max
        
        for point in points:
            if point['distance'] <= 0.01 or point['distance'] > self.range_max:
                continue
            
            # Convert polar to cartesian
            angle_rad = np.radians(point['azimuth'])
            x = center + int(np.cos(angle_rad) * point['distance'] * pixels_per_meter)
            y = center - int(np.sin(angle_rad) * point['distance'] * pixels_per_meter)
            
            if 0 <= x < canvas_size and 0 <= y < canvas_size:
                # Color based on RSSI
                intensity = min(255, point.get('rssi', 128))
                cv2.circle(self.image, (x, y), 1, (0, intensity, 0), -1)
        
        # Draw range circles
        for r in [2, 4, 6, 8]:
            if r <= self.range_max:
                radius = int(r * pixels_per_meter)
                cv2.circle(self.image, (center, center), radius, (50, 50, 50), 1)
        
        # Draw crosshairs
        cv2.line(self.image, (center, 0), (center, canvas_size), (100, 100, 100), 1)
        cv2.line(self.image, (0, center), (canvas_size, center), (100, 100, 100), 1)
        
        self.update()
    
    def paintEvent(self, event):
        # Convert numpy array to QImage and display
        height, width, channel = self.image.shape
        bytes_per_line = 3 * width
        q_image = QImage(self.image.data, width, height, bytes_per_line, QImage.Format_RGB888)
        
        # Scale to widget size
        pixmap = QPixmap.fromImage(q_image)
        pixmap = pixmap.scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        
        from PyQt5.QtGui import QPainter
        painter = QPainter(self)
        painter.drawPixmap(0, 0, pixmap)

class RobotTelemetryInterface(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Telemetry Interface")
        self.setGeometry(100, 100, 1600, 1000)
        
        # Data storage for plotting
        self.max_points = 1000
        self.time_data = deque(maxlen=self.max_points)
        self.velocity_data = deque(maxlen=self.max_points)
        self.omega_data = deque(maxlen=self.max_points)
        self.expected_vel_data = deque(maxlen=self.max_points)
        self.expected_omega_data = deque(maxlen=self.max_points)
        self.start_time = time.time()
        
        # Current sensor values
        self.current_sensor = {}
        self.current_debug = {}
        self.current_echo = {}
        
        self.initUI()
        self.setup_communications()
        
        # Update timer for plots
        self.plot_timer = QTimer()
        self.plot_timer.timeout.connect(self.update_plots)
        self.plot_timer.start(100)  # 10 Hz update rate
    
    def initUI(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Main layout
        main_layout = QHBoxLayout()
        
        # Left panel - Controls and data displays
        left_panel = self.create_left_panel()
        
        # Right panel - Plots and visualization
        right_panel = self.create_right_panel()
        
        # Add panels to main layout
        main_layout.addWidget(left_panel, 1)
        main_layout.addWidget(right_panel, 2)
        
        central_widget.setLayout(main_layout)
    
    def create_left_panel(self):
        left_widget = QWidget()
        left_layout = QVBoxLayout()
        
        # Connection controls
        conn_group = QGroupBox("Connection")
        conn_layout = QGridLayout()
        
        conn_layout.addWidget(QLabel("Listen IP:"), 0, 0)
        self.listen_ip = QLineEdit("192.168.8.157")
        conn_layout.addWidget(self.listen_ip, 0, 1)
        
        conn_layout.addWidget(QLabel("Listen Port:"), 1, 0)
        self.listen_port = QLineEdit("9001")
        conn_layout.addWidget(self.listen_port, 1, 1)
        
        conn_layout.addWidget(QLabel("Robot IP:"), 2, 0)
        self.robot_ip = QLineEdit("192.168.8.143")
        conn_layout.addWidget(self.robot_ip, 2, 1)
        
        conn_layout.addWidget(QLabel("Robot Port:"), 3, 0)
        self.robot_port = QLineEdit("9000")
        conn_layout.addWidget(self.robot_port, 3, 1)
        
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)
        conn_layout.addWidget(self.connect_btn, 4, 0, 1, 2)
        
        conn_group.setLayout(conn_layout)
        left_layout.addWidget(conn_group)
        
        # Sensor data display
        sensor_group = QGroupBox("Sensor Data")
        sensor_layout = QGridLayout()
        
        self.sensor_labels = {}
        sensor_fields = ['yaw', 'roll', 'pitch', 'gyroX', 'gyroY', 'gyroZ', 
                        'accelX', 'accelY', 'accelZ', 'encL', 'encR', 
                        'vbat1', 'vbat2', 'emergency', 'profileDone']
        
        for i, field in enumerate(sensor_fields):
            row = i // 2
            col = (i % 2) * 2
            sensor_layout.addWidget(QLabel(f"{field}:"), row, col)
            label = QLabel("0.00")
            label.setStyleSheet("QLabel { background-color: #f0f0f0; padding: 2px; }")
            self.sensor_labels[field] = label
            sensor_layout.addWidget(label, row, col + 1)
        
        sensor_group.setLayout(sensor_layout)
        left_layout.addWidget(sensor_group)
        
        # Motion data display
        motion_group = QGroupBox("Motion Data")
        motion_layout = QGridLayout()
        
        self.motion_labels = {}
        motion_fields = ['vel', 'omg', 'spdL', 'spdR', 'dist', 'ang', 'loopDt']
        
        for i, field in enumerate(motion_fields):
            motion_layout.addWidget(QLabel(f"{field}:"), i, 0)
            label = QLabel("0.00")
            label.setStyleSheet("QLabel { background-color: #f0f0f0; padding: 2px; }")
            self.motion_labels[field] = label
            motion_layout.addWidget(label, i, 1)
        
        motion_group.setLayout(motion_layout)
        left_layout.addWidget(motion_group)
        
        # Command controls
        cmd_group = QGroupBox("Command Controls")
        cmd_layout = QGridLayout()
        
        cmd_layout.addWidget(QLabel("Mode:"), 0, 0)
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["STOP", "AUTONOMOUS", "TELEOPERATOR", "PROFILE"])
        cmd_layout.addWidget(self.mode_combo, 0, 1)
        
        cmd_layout.addWidget(QLabel("Debug:"), 1, 0)
        self.debug_combo = QComboBox()
        self.debug_combo.addItems(["NONE", "RX_ECHO", "MD_AND_ECHO"])
        cmd_layout.addWidget(self.debug_combo, 1, 1)
        
        # Command parameters
        self.cmd_params = {}
        params = ['distance', 'angle', 'maxVel', 'maxOmega', 'lastVel', 'lastOmega', 'linAcc', 'angAcc']
        for i, param in enumerate(params):
            row = 2 + i // 2
            col = (i % 2) * 2
            cmd_layout.addWidget(QLabel(f"{param}:"), row, col)
            line_edit = QLineEdit("0.0")
            self.cmd_params[param] = line_edit
            cmd_layout.addWidget(line_edit, row, col + 1)
        
        send_cmd_btn = QPushButton("Send Command")
        send_cmd_btn.clicked.connect(self.send_command)
        cmd_layout.addWidget(send_cmd_btn, row + 1, 0, 1, 4)
        
        cmd_group.setLayout(cmd_layout)
        left_layout.addWidget(cmd_group)
        
        # Teleop controls info
        teleop_group = QGroupBox("Teleop (Keyboard)")
        teleop_layout = QVBoxLayout()
        teleop_layout.addWidget(QLabel("W - Forward"))
        teleop_layout.addWidget(QLabel("S - Backward"))
        teleop_layout.addWidget(QLabel("A - Left"))
        teleop_layout.addWidget(QLabel("D - Right"))
        teleop_layout.addWidget(QLabel("Q - Quit"))
        teleop_group.setLayout(teleop_layout)
        left_layout.addWidget(teleop_group)
        
        # Message log
        log_group = QGroupBox("Message Log")
        log_layout = QVBoxLayout()
        self.message_log = QTextEdit()
        self.message_log.setMaximumHeight(150)
        self.message_log.setReadOnly(True)
        log_layout.addWidget(self.message_log)
        log_group.setLayout(log_layout)
        left_layout.addWidget(log_group)
        
        left_layout.addStretch()
        left_widget.setLayout(left_layout)
        return left_widget
    
    def create_right_panel(self):
        right_widget = QWidget()
        right_layout = QVBoxLayout()
        
        # Create tab widget for different visualizations
        tab_widget = QTabWidget()
        
        # Velocity plots tab
        velocity_tab = QWidget()
        velocity_layout = QVBoxLayout()
        
        # Velocity plot
        self.velocity_plot = pg.PlotWidget()
        self.velocity_plot.setBackground('w')
        self.velocity_plot.setTitle("Velocity Profile", color='b', size="12pt")
        self.velocity_plot.setLabel("left", "Velocity (m/s)", color='blue')
        self.velocity_plot.setLabel("bottom", "Time (s)", color='blue')
        self.velocity_plot.showGrid(x=True, y=True)
        self.velocity_plot.addLegend()
        
        # Plot items
        self.vel_curve = self.velocity_plot.plot(pen=pg.mkPen(color='red', width=2), name='Actual Velocity')
        self.expected_vel_curve = self.velocity_plot.plot(pen=pg.mkPen(color='blue', width=2, style=pg.QtCore.Qt.DashLine), name='Expected Velocity')
        
        velocity_layout.addWidget(self.velocity_plot)
        
        # Angular velocity plot
        self.omega_plot = pg.PlotWidget()
        self.omega_plot.setBackground('w')
        self.omega_plot.setTitle("Angular Velocity Profile", color='b', size="12pt")
        self.omega_plot.setLabel("left", "Angular Velocity (rad/s)", color='blue')
        self.omega_plot.setLabel("bottom", "Time (s)", color='blue')
        self.omega_plot.showGrid(x=True, y=True)
        self.omega_plot.addLegend()
        
        self.omega_curve = self.omega_plot.plot(pen=pg.mkPen(color='green', width=2), name='Actual Omega')
        self.expected_omega_curve = self.omega_plot.plot(pen=pg.mkPen(color='magenta', width=2, style=pg.QtCore.Qt.DashLine), name='Expected Omega')
        
        velocity_layout.addWidget(self.omega_plot)
        
        velocity_tab.setLayout(velocity_layout)
        tab_widget.addTab(velocity_tab, "Velocity Profiles")
        
        # LiDAR visualization tab
        lidar_tab = QWidget()
        lidar_layout = QVBoxLayout()
        
        self.lidar_visualizer = LidarVisualizer()
        lidar_layout.addWidget(QLabel("LiDAR Visualization (10m range)"))
        lidar_layout.addWidget(self.lidar_visualizer)
        
        lidar_tab.setLayout(lidar_layout)
        tab_widget.addTab(lidar_tab, "LiDAR")
        
        right_layout.addWidget(tab_widget)
        right_widget.setLayout(right_layout)
        return right_widget
    
    def setup_communications(self):
        self.udp_handler = None
        self.keyboard_controller = None
        self.connected = False
    
    def toggle_connection(self):
        if not self.connected:
            # Connect
            try:
                self.udp_handler = UDPHandler(
                    self.listen_ip.text(),
                    int(self.listen_port.text()),
                    self.robot_ip.text(),
                    int(self.robot_port.text())
                )
                
                # Connect signals
                self.udp_handler.sensor_data.connect(self.update_sensor_data)
                self.udp_handler.debug_data.connect(self.update_debug_data)
                self.udp_handler.echo_data.connect(self.update_echo_data)
                self.udp_handler.lidar_data.connect(self.update_lidar_data)
                self.udp_handler.velocity_profile_data.connect(self.update_velocity_profile_data)
                self.udp_handler.raw_message.connect(self.log_message)
                
                self.udp_handler.start()
                
                # Start keyboard controller
                self.keyboard_controller = KeyboardController()
                self.keyboard_controller.key_pressed.connect(self.handle_keypress)
                self.keyboard_controller.start()
                
                self.connect_btn.setText("Disconnect")
                self.connected = True
                self.log_message("Connected to robot")
                
            except Exception as e:
                self.log_message(f"Connection failed: {e}")
        else:
            # Disconnect
            if self.udp_handler:
                self.udp_handler.stop()
                self.udp_handler = None
            
            if self.keyboard_controller:
                self.keyboard_controller.stop()
                self.keyboard_controller = None
            
            self.connect_btn.setText("Connect")
            self.connected = False
            self.log_message("Disconnected from robot")
    
    def handle_keypress(self, key):
        if not self.udp_handler:
            return
        
        f = b = l = r = 0
        if key == 'w':
            f = 1
        elif key == 's':
            b = 1
        elif key == 'a':
            l = 1
        elif key == 'd':
            r = 1
        elif key == 'q':
            self.close()
            return
        else:
            return
        
        self.udp_handler.send_teleop(f, b, l, r)
    
    def send_command(self):
        if not self.udp_handler:
            return
        
        cmd_dict = {
            'mode': self.mode_combo.currentIndex(),
            'debug': self.debug_combo.currentIndex()
        }
        
        for param, line_edit in self.cmd_params.items():
            try:
                cmd_dict[param] = float(line_edit.text())
            except ValueError:
                cmd_dict[param] = 0.0
        
        self.udp_handler.send_command(cmd_dict)
    
    def update_sensor_data(self, data):
        self.current_sensor = data
        for field, value in data.items():
            if field in self.sensor_labels:
                if isinstance(value, float):
                    self.sensor_labels[field].setText(f"{value:.3f}")
                else:
                    self.sensor_labels[field].setText(str(value))
    
    def update_debug_data(self, data):
        self.current_debug = data
        
        # Update motion labels
        for field, value in data.items():
            if field in self.motion_labels:
                self.motion_labels[field].setText(f"{value:.3f}")
        
        # Add data to plots
        current_time = time.time() - self.start_time
        self.time_data.append(current_time)
        self.velocity_data.append(data.get('vel', 0))
        self.omega_data.append(data.get('omg', 0))
    
    def update_lidar_data(self, data):
        """Update LiDAR visualization"""
        points = data.get('points', [])
        self.lidar_visualizer.update_lidar_data(points)
    
    def update_velocity_profile_data(self, data):
        """Update velocity profile data from VELOCITY_PROFILE packets"""
        # This provides both actual and expected velocities in one packet
        current_time = time.time() - self.start_time
        self.time_data.append(current_time)
        
        # Actual values
        self.velocity_data.append(data.get('actual_vel', 0))
        self.omega_data.append(data.get('actual_omega', 0))
        
        # Expected values
        self.expected_vel_data.append(data.get('expected_vel', 0))
        self.expected_omega_data.append(data.get('expected_omega', 0))
        
        # Update motion labels with additional data
        if 'spdL' in data and 'spdL' in self.motion_labels:
            self.motion_labels['spdL'].setText(f"{data['spdL']:.3f}")
        if 'spdR' in data and 'spdR' in self.motion_labels:
            self.motion_labels['spdR'].setText(f"{data['spdR']:.3f}")
        if 'loop_dt' in data and 'loopDt' in self.motion_labels:
            self.motion_labels['loopDt'].setText(f"{data['loop_dt']:.3f}")
    
    def update_echo_data(self, data):
        self.current_echo = data
        # ECHO data now handled by VELOCITY_PROFILE packets for better synchronization
    
    def update_plots(self):
        if len(self.time_data) > 1:
            time_array = np.array(self.time_data)
            
            # Update velocity plot
            vel_array = np.array(self.velocity_data)
            self.vel_curve.setData(time_array, vel_array)
            
            if len(self.expected_vel_data) > 0:
                expected_vel_array = np.array(self.expected_vel_data)
                # Ensure arrays have same length
                min_len = min(len(time_array), len(expected_vel_array))
                self.expected_vel_curve.setData(time_array[-min_len:], expected_vel_array[-min_len:])
            
            # Update omega plot
            omega_array = np.array(self.omega_data)
            self.omega_curve.setData(time_array, omega_array)
            
            if len(self.expected_omega_data) > 0:
                expected_omega_array = np.array(self.expected_omega_data)
                min_len = min(len(time_array), len(expected_omega_array))
                self.expected_omega_curve.setData(time_array[-min_len:], expected_omega_array[-min_len:])
    
    def log_message(self, message):
        timestamp = time.strftime("%H:%M:%S")
        self.message_log.append(f"[{timestamp}] {message}")
        
        # Limit log size
        if self.message_log.document().blockCount() > 100:
            cursor = self.message_log.textCursor()
            cursor.movePosition(cursor.Start)
            cursor.select(cursor.BlockUnderCursor)
            cursor.removeSelectedText()
    
    def closeEvent(self, event):
        if self.connected:
            self.toggle_connection()
        event.accept()

def main():
    app = QApplication(sys.argv)
    
    # Set application style
    app.setStyle('Fusion')
    
    window = RobotTelemetryInterface()
    window.show()
    
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()