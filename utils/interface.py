import sys
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QWidget, QTextEdit, QPushButton,
    QComboBox, QLabel, QHBoxLayout, QCheckBox, QLineEdit, QSplitter
)
from PyQt5.QtCore import QThread, pyqtSignal, Qt
import pyqtgraph as pg
import socket


class UDPReaderThread(QThread):
    data_received = pyqtSignal(str)

    def __init__(self, ip, port):
        super().__init__()
        self.ip = ip
        self.port = port
        self.running = True

    def run(self):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind((self.ip, self.port))
            while self.running:
                data, _ = sock.recvfrom(2048)
                try:
                    message = data.decode('utf-8').strip()
                except UnicodeDecodeError:
                    message = f"Received non-UTF-8 data: {data}"
                self.data_received.emit(message)
        except Exception as e:
            self.data_received.emit(f"UDP Error: {e}")
        finally:
            sock.close()

    def stop(self):
        self.running = False
        self.wait()


class SerialMonitor(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Serial/UDP Monitor")
        self.setGeometry(100, 100, 1200, 800)

        self.udp_thread = None
        self.plot_items = {}
        self.header_checkboxes = {}  

        # Data series for graph
        self.data_series = {
            "SPD": [], "ROT_SPD": [], "EXP_SPD": [], "EXP_ROT": [],
            "LFT_FDSPD": [], "RT_FDSPD": [], "LR_SPD": [], "RR_SPD": [],
        }

        # Current numeric displays
        self.current_values = {
            "Current Speed": 0.0,
            "Omega": 0.0,
            "Expected Speed": 0.0,
            "Expected Omega": 0.0,
            "Left Front Speed": 0.0,
            "Right Front Speed": 0.0,
            "Left Rear Speed": 0.0,
            "Right Rear Speed": 0.0,
        }

        self.plot_items = {}
        self.initUI()

    def initUI(self):
        main_widget = QWidget(self)
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout()

        # Left panel: controls
        left_panel = QWidget()
        left_layout = QVBoxLayout()
        left_panel.setLayout(left_layout)
        left_panel.setMaximumWidth(400)

        # — UDP controls —
        udp_group = QWidget()
        udp_layout = QVBoxLayout()

        read_udp = QHBoxLayout()
        self.ip_label = QLabel("Read IP:")
        self.ip_input = QComboBox()
        self.ip_input.addItems(["0.0.0.0", "192.168.1.68"])
        self.port_label = QLabel("Port:")
        self.port_input = QComboBox()
        self.port_input.addItems(["9001", "5005", "9000"])
        read_udp.addWidget(self.ip_label)
        read_udp.addWidget(self.ip_input)
        read_udp.addWidget(self.port_label)
        read_udp.addWidget(self.port_input)
        udp_layout.addLayout(read_udp)

        send_udp = QHBoxLayout()
        self.send_ip_label = QLabel("Send IP:")
        self.send_ip_input = QLineEdit("192.168.1.84")
        self.send_port_label = QLabel("Port:")
        self.send_port_input = QLineEdit("9000")
        send_udp.addWidget(self.send_ip_label)
        send_udp.addWidget(self.send_ip_input)
        send_udp.addWidget(self.send_port_label)
        send_udp.addWidget(self.send_port_input)
        udp_layout.addLayout(send_udp)

        buttons = QHBoxLayout()
        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.connect_to_udp)
        self.stop_button = QPushButton("Stop")
        self.stop_button.setEnabled(False)
        self.stop_button.clicked.connect(self.stop_reading)
        buttons.addWidget(self.connect_button)
        buttons.addWidget(self.stop_button)
        udp_layout.addLayout(buttons)

        udp_group.setLayout(udp_layout)
        left_layout.addWidget(udp_group)

        # — Real-time values —
        values_group = QWidget()
        value_layout = QVBoxLayout()
        self.value_labels = {}
        for key in self.current_values:
            label = QLabel(f"{key}: 0.0")
            self.value_labels[key] = label
            value_layout.addWidget(label)
        values_group.setLayout(value_layout)
        left_layout.addWidget(values_group)

        # — Parameter inputs —
        params_group = QWidget()
        param_layout = QVBoxLayout()
        self.params = {}
        for param in ["FWD_KP", "FWD_KD", "FWD_KI", "ROT_KP", "ROT_KD", "ROT_KI"]:
            row_layout = QHBoxLayout()
            label = QLabel(f"{param}:")
            input_field = QLineEdit()
            input_field.setPlaceholderText("Enter value")
            button = QPushButton("Send")
            button.clicked.connect(lambda _, p=param, i=input_field: self.send_param_udp(p, i))
            row_layout.addWidget(label)
            row_layout.addWidget(input_field)
            row_layout.addWidget(button)
            param_layout.addLayout(row_layout)
            self.params[param] = input_field
        params_group.setLayout(param_layout)
        left_layout.addWidget(params_group)

        # — Other messages —
        self.other_output = QTextEdit()
        self.other_output.setReadOnly(True)
        self.other_output.setPlaceholderText("Other Messages")
        self.other_output.setMaximumHeight(100)
        left_layout.addWidget(QLabel("Messages:"))
        left_layout.addWidget(self.other_output)
        left_layout.addStretch()

        # Right panel: graph
        right_panel = QWidget()
        right_layout = QVBoxLayout()
        right_panel.setLayout(right_layout)

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground('w')
        self.plot_widget.setTitle("Real-Time Data Plot", color='b', size="12pt")
        self.plot_widget.setLabel("left", "Value", color='blue')
        self.plot_widget.setLabel("bottom", "Sample", color='blue')
        self.plot_widget.showGrid(x=True, y=True)
        right_layout.addWidget(self.plot_widget)

        plot_controls = QHBoxLayout()
        colors = {
            "SPD": "r", "ROT_SPD": "g", "EXP_SPD": "b", "EXP_ROT": "m",
            "LFT_FDSPD": "y", "RT_FDSPD": "c", "LR_SPD": (255,128,0), "RR_SPD": (128,0,128)
        }
        for header, color in colors.items():
            cb = QCheckBox(header)
            cb.setChecked(True)
            cb.stateChanged.connect(self.update_plot_visibility)
            self.header_checkboxes[header] = cb
            self.plot_items[header] = self.plot_widget.plot(pen=pg.mkPen(color=color, width=2), name=header)
            plot_controls.addWidget(cb)
        right_layout.addLayout(plot_controls)

        main_layout.addWidget(left_panel)
        main_layout.addWidget(right_panel, stretch=2)
        main_widget.setLayout(main_layout)

    def update_output(self, data):
        # ---- handle SENSOR messages ----
        if data.startswith("SENSOR,"):
            parts = data.split(",")[1:]
            try:
                yaw, roll, pitch = map(float, parts[0:3])
                # you can display or log these if desired...
                self.other_output.append(f"Yaw={yaw:.1f}, Roll={roll:.1f}, Pitch={pitch:.1f}")
            except:
                self.other_output.append(f"Malformed SENSOR: {data}")

        # ---- handle DEBUG messages ----
        elif data.startswith("DEBUG,"):
            parts = data.split(",")[1:]
            try:
                spdL, spdR, vel, omg, dist, ang, dt = map(float, parts)
                # update your graph series:
                self.data_series["SPD"].append(vel)
                self.data_series["ROT_SPD"].append(omg)
                # trim
                for key in ["SPD","ROT_SPD"]:
                    if len(self.data_series[key])>2000:
                        self.data_series[key].pop(0)
                # update labels
                self.value_labels["Current Speed"].setText(f"Current Speed: {vel:.2f}")
                self.value_labels["Omega"].setText(f"Omega: {omg:.2f}")
                # update plots
                self.update_plot("SPD")
                self.update_plot("ROT_SPD")
            except:
                self.other_output.append(f"Malformed DEBUG: {data}")

        # ---- existing VELOCITY: branch ----
        elif data.startswith("VELOCITY:"):
            try:
                parts = data.split(":")[1].split(",")
                exp_spd, spd, exp_rot, rot_spd, lft_fdspd, rt_fdspd, lr_spd, rr_spd = map(float, parts)
                series = {
                  "EXP_SPD": exp_spd, "SPD": spd,
                  "EXP_ROT": exp_rot, "ROT_SPD": rot_spd,
                  "LFT_FDSPD": lft_fdspd, "RT_FDSPD": rt_fdspd,
                  "LR_SPD": lr_spd, "RR_SPD": rr_spd
                }
                for key,val in series.items():
                    self.data_series[key].append(val)
                    if len(self.data_series[key])>2000: self.data_series[key].pop(0)
                    self.value_labels[key.replace("_"," ").title()].setText(f"{key.replace('_',' ')}: {val:.2f}")
                    self.update_plot(key)
            except Exception as e:
                self.other_output.append(f"Invalid VELOCITY format: {data}")

        else:
            self.other_output.append(data)

    def connect_to_udp(self):
        ip   = self.ip_input.currentText()
        port = int(self.port_input.currentText())
        try:
            self.udp_thread = UDPReaderThread(ip, port)
            self.udp_thread.data_received.connect(self.update_output)
            self.udp_thread.start()
            self.other_output.append(f"Listening on {ip}:{port}")
            self.connect_button.setEnabled(False)
            self.stop_button.setEnabled(True)
        except Exception as e:
            self.other_output.append(f"UDP Connection Error: {e}")

    def stop_reading(self):
        if self.udp_thread:
            self.udp_thread.stop()
            self.udp_thread = None
        self.other_output.append("Stopped reading.")
        self.connect_button.setEnabled(True)
        self.stop_button.setEnabled(False)

    def update_plot(self, header):
        if self.header_checkboxes[header].isChecked():
            self.plot_items[header].setData(self.data_series[header])
        else:
            self.plot_items[header].clear()

    def update_plot_visibility(self):
        for header, cb in self.header_checkboxes.items():
            if not cb.isChecked():
                self.plot_items[header].clear()
            elif self.data_series[header]:
                self.plot_items[header].setData(self.data_series[header])

    def send_param_udp(self, param, input_field):
        value = input_field.text()
        if value:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                send_ip   = self.send_ip_input.text()
                send_port = int(self.send_port_input.text())
                message = f"{param}={value}"
                sock.sendto(message.encode('utf-8'), (send_ip, send_port))
                self.other_output.append(f"Sent: {message} to {send_ip}:{send_port}")
            except Exception as e:
                self.other_output.append(f"Error sending {param}: {e}")
        else:
            self.other_output.append(f"No value for {param}")

def main():
    app = QApplication(sys.argv)
    window = SerialMonitor()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
