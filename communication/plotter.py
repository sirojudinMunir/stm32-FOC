import sys
import struct
import numpy as np
import pyqtgraph as pg
from PyQt5 import QtCore, QtWidgets
from collections import deque
import serial
import serial.tools.list_ports
import threading
import queue
import time
import io
import contextlib
import traceback
from motor_protocol import MotorProtocol
import queue
import colorsys

class SerialPortDialog(QtWidgets.QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Select Serial Port")
        self.setModal(True)
        self.setFixedWidth(400)
        
        layout = QtWidgets.QVBoxLayout(self)
        
        # Label
        label = QtWidgets.QLabel("Select Serial Port:")
        layout.addWidget(label)
        
        # Combo box untuk port
        self.port_combo = QtWidgets.QComboBox()
        self.port_combo.setPlaceholderText("Select port...")
        layout.addWidget(self.port_combo)
        
        # Refresh button
        refresh_btn = QtWidgets.QPushButton("Refresh Ports")
        refresh_btn.clicked.connect(self.refresh_ports)
        layout.addWidget(refresh_btn)
        
        # Baudrate
        baud_label = QtWidgets.QLabel("Baudrate:")
        layout.addWidget(baud_label)
        
        self.baud_combo = QtWidgets.QComboBox()
        self.baud_combo.addItems(['9600', '19200', '38400', '57600', '115200', '230400', '460800'])
        self.baud_combo.setCurrentText('115200')
        layout.addWidget(self.baud_combo)
        
        # Buttons
        button_box = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel
        )
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)
        layout.addWidget(button_box)
        
        # Refresh ports on show
        self.refresh_ports()
        
        # Auto-select jika hanya ada satu port
        if self.port_combo.count() == 1:
            self.port_combo.setCurrentIndex(0)
    
    def refresh_ports(self):
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        
        if not ports:
            self.port_combo.addItem("No ports found")
            self.port_combo.setEnabled(False)
        else:
            self.port_combo.setEnabled(True)
            for port in ports:
                description = f"{port.device} - {port.description}"
                self.port_combo.addItem(description, port.device)
            
            self.port_combo.setCurrentIndex(0)
    
    def get_selected_port(self):
        if self.port_combo.currentIndex() >= 0:
            return self.port_combo.currentData()
        return None
    
    def get_baudrate(self):
        return int(self.baud_combo.currentText())


class DataAcquisitionThread(QtCore.QThread):
    data_received = QtCore.pyqtSignal(list)
    connection_lost = QtCore.pyqtSignal()
    
    def __init__(self, serial_conn=None, parent=None):
        super().__init__(parent)
        self.running = True
        self.serial_conn = serial_conn
        self.data_queue = queue.Queue(maxsize=10000)
        self.raw_buffer = bytearray()
        self.response_queue = queue.Queue()
        self.expected_response_size = None
        self.error_count = 0
        self.max_errors = 10
        
    def run(self):
        while self.running:
            try:
                if self.serial_conn and self.serial_conn.is_open:
                    raw_data = self.serial_conn.read(self.serial_conn.in_waiting)
                    if raw_data:
                        self.raw_buffer.extend(raw_data)
                        self.error_count = 0  # Reset error count on successful read
                
                self.parse_buffer()
                QtCore.QThread.msleep(1)
                
            except serial.SerialException as e:
                self.error_count += 1
                print(f"Serial error: {e}")
                if self.error_count >= self.max_errors:
                    print("Too many serial errors, emitting connection_lost")
                    self.connection_lost.emit()
                    break
                QtCore.QThread.msleep(100)
            except Exception as e:
                print(f"Error di thread akuisisi: {e}")
                QtCore.QThread.msleep(10)
    
    def expect_response(self, size):
        self.expected_response_size = size

    def parse_buffer(self):
        while True:
            if len(self.raw_buffer) < 3:
                break

            header = struct.unpack("<H", self.raw_buffer[:2])[0]
            # LIVE PLOT
            if header == 0xABCD:
                num_channel = self.raw_buffer[2]
                frame_size = 3 + num_channel * 4
                if len(self.raw_buffer) < frame_size:
                    break
                values = []
                offset = 3
                for _ in range(num_channel):
                    value = struct.unpack(
                        "<f",
                        self.raw_buffer[offset:offset+4]
                    )[0]
                    values.append(value)
                    offset += 4
                self.data_received.emit(values)
                self.raw_buffer = self.raw_buffer[frame_size:]

            # COMMAND RESPONSE
            elif header == 0xA55A:
                if self.expected_response_size is None:
                    break
                frame_size = 2 + self.expected_response_size
                if len(self.raw_buffer) < frame_size:
                    break
                payload = bytes(self.raw_buffer[2:frame_size])
                self.response_queue.put(payload)
                self.raw_buffer = self.raw_buffer[frame_size:]
                self.expected_response_size = None

            else:
                self.raw_buffer.pop(0)
    
    def stop(self):
        self.running = False
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        self.wait()


class LivePlotter(QtWidgets.QMainWindow):
    def __init__(self, max_points=1000, port=None, baudrate=115200):
        super().__init__()
        
        self.max_points = max_points
        self.serial_conn = None
        self.acq_thread = None
        self.is_connected = False
        
        # Setup UI
        self.setup_ui()
        
        # Buffer untuk data
        self.time_buffer = deque(maxlen=max_points)
        self.data_buffers = []  # Akan diisi sesuai channel
        self.counter = 0
        
        # Timer untuk update plot
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(10)  # Update setiap 10ms (100 FPS)
        
        # Setup performance timer
        self.last_update = time.time()
        self.fps_counter = 0
        self.fps_result = 0
        self.fps_timer = QtCore.QTimer()
        self.fps_timer.timeout.connect(self.update_fps)
        self.fps_timer.start(1000)  # Update FPS setiap 1 detik
        
        # Status
        self.status_label = QtWidgets.QLabel("Not Connected")
        self.statusBar().addWidget(self.status_label)
        
        # Console namespace
        self.console_namespace = {
            "plotter": self,
            "thread": self.acq_thread,
            "motor": self.motor if hasattr(self, 'motor') else None,
            "np": np,
            "pg": pg,
        }
        self.setup_console_completion()

        self.channels = {}
        self.used_colors = set()
        self.available_colors = [
            (255, 0, 0),      # Red
            (0, 0, 255),      # Blue
            (0, 255, 0),      # Green
            (255, 165, 0),    # Orange
            (128, 0, 128),    # Purple
            (255, 192, 203),  # Pink
            (0, 255, 255),    # Cyan
            (255, 0, 255),    # Magenta
            (255, 255, 0),    # Yellow
            (0, 128, 128),    # Teal
            (128, 128, 0),    # Olive
            (128, 0, 0),      # Maroon
            (0, 0, 128),      # Navy
            (0, 128, 0),      # Forest Green
        ]
        self.color_index = 0

        # Try auto-connect if port specified
        if port:
            self.connect_serial(port, baudrate)
        else:
            # Show connection dialog after UI is ready
            QtCore.QTimer.singleShot(100, self.show_connection_dialog)
        
        print(f"LivePlotter initialized")
    
    def setup_ui(self):
        self.setWindowTitle('Live Plotter - PyQtGraph')
        self.setGeometry(100, 100, 1200, 600)
        
        # Central widget
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        layout = QtWidgets.QVBoxLayout(central_widget)
        
        # Toolbar
        toolbar = QtWidgets.QToolBar()
        self.addToolBar(toolbar)
        
        # Connect button
        self.connect_action = QtWidgets.QAction('Connect', self)
        self.connect_action.triggered.connect(self.show_connection_dialog)
        toolbar.addAction(self.connect_action)
        
        # Disconnect button
        self.disconnect_action = QtWidgets.QAction('Disconnect', self)
        self.disconnect_action.triggered.connect(self.disconnect_serial)
        self.disconnect_action.setEnabled(False)
        toolbar.addAction(self.disconnect_action)
        
        toolbar.addSeparator()
        
        # Plot widget
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground('w')
        self.plot_widget.setLabel('left', 'Value')
        self.plot_widget.setLabel('bottom', 'Sample')
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.addLegend()
        
        layout.addWidget(self.plot_widget)
        
        # Control buttons
        control_layout = QtWidgets.QHBoxLayout()
        
        self.clear_button = QtWidgets.QPushButton('Clear')
        self.clear_button.clicked.connect(self.clear_data)
        control_layout.addWidget(self.clear_button)
        
        self.pause_button = QtWidgets.QPushButton('Pause')
        self.pause_button.clicked.connect(self.toggle_pause)
        self.paused = False
        control_layout.addWidget(self.pause_button)
        
        self.auto_range_check = QtWidgets.QCheckBox('Auto Range')
        self.auto_range_check.setChecked(True)
        control_layout.addWidget(self.auto_range_check)
        
        control_layout.addStretch()
        
        # Status info
        self.info_label = QtWidgets.QLabel('Samples: 0 | FPS: 0')
        control_layout.addWidget(self.info_label)
        
        layout.addLayout(control_layout)

        # Console output
        self.console_output = QtWidgets.QPlainTextEdit()
        self.console_output.setReadOnly(True)
        self.console_output.setMaximumHeight(180)
        layout.addWidget(self.console_output)

        # Command line
        self.console_input = QtWidgets.QLineEdit()
        self.console_input.setPlaceholderText(">>>")
        self.console_input.returnPressed.connect(self.execute_command)
        layout.addWidget(self.console_input)

    def setup_console_completion(self):
        words = self.build_completion()
        completer = QtWidgets.QCompleter(words, self)
        completer.setCompletionMode(
            QtWidgets.QCompleter.PopupCompletion
        )
        completer.setCaseSensitivity(
            QtCore.Qt.CaseInsensitive
        )
        completer.setFilterMode(
            QtCore.Qt.MatchContains
        )
        self.console_input.setCompleter(completer)

    def build_completion(self):
        words = []
        for name, obj in self.console_namespace.items():
            if obj is not None:
                words.append(name)
                for attr in dir(obj):
                    if attr.startswith("_"):
                        continue
                    words.append(f"{name}.{attr}")
        return sorted(words)
        
    def execute_command(self):
        cmd = self.console_input.text()
        self.console_output.appendPlainText(f">>> {cmd}")
        try:
            buffer = io.StringIO()
            with contextlib.redirect_stdout(buffer):
                try:
                    result = eval(cmd, globals(), self.console_namespace)
                    if result is not None:
                        print(result)
                except SyntaxError:
                    exec(cmd, globals(), self.console_namespace)
            out = buffer.getvalue()
            if out:
                self.console_output.appendPlainText(out)
        except Exception:
            self.console_output.appendPlainText(
                traceback.format_exc()
            )
        self.console_input.clear()
    
    def show_connection_dialog(self):
        dialog = SerialPortDialog(self)
        if dialog.exec_() == QtWidgets.QDialog.Accepted:
            port = dialog.get_selected_port()
            baudrate = dialog.get_baudrate()
            if port:
                self.connect_serial(port, baudrate)
    
    def connect_serial(self, port, baudrate):
        try:
            # Close existing connection if any
            self.disconnect_serial()
            
            # Create new serial connection
            self.serial_conn = serial.Serial(port, baudrate, timeout=0.001)
            print(f"Connected to {port} at {baudrate} baud")
            
            # Start acquisition thread
            self.acq_thread = DataAcquisitionThread(serial_conn=self.serial_conn)
            self.acq_thread.data_received.connect(self.on_data_received)
            self.acq_thread.connection_lost.connect(self.handle_connection_lost)
            self.acq_thread.start()
            
            # Initialize motor protocol
            self.motor = MotorProtocol(self.serial_conn, self.acq_thread)
            
            # Update console namespace
            self.console_namespace["thread"] = self.acq_thread
            self.console_namespace["motor"] = self.motor
            self.setup_console_completion()
            
            # Update UI
            self.is_connected = True
            self.connect_action.setEnabled(False)
            self.disconnect_action.setEnabled(True)
            self.status_label.setText(f"Connected to {port} at {baudrate} baud")
            
            print("Serial connection established successfully")

            # setup plotter
            motor_mode = self.motor.get_foc_motor_mode()
            if motor_mode == 0:
                self.motor.plotter_add_line('Is_ref')
                self.motor.plotter_add_line('id')
                self.motor.plotter_add_line('iq')
            elif motor_mode == 1:
                self.motor.plotter_add_line('rpm_ref')
                self.motor.plotter_add_line('actual_rpm')
            elif motor_mode == 2:
                self.motor.plotter_add_line('pos_ref')
                self.motor.plotter_add_line('actual_angle')

            
        except serial.SerialException as e:
            QtWidgets.QMessageBox.critical(
                self,
                "Connection Error",
                f"Failed to connect to {port}:\n{str(e)}"
            )
            self.status_label.setText(f"Connection failed: {str(e)}")
            print(f"Connection error: {e}")
    
    def disconnect_serial(self):
        if self.acq_thread:
            self.acq_thread.stop()
            self.acq_thread = None
        
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            self.serial_conn = None
        
        self.is_connected = False
        self.connect_action.setEnabled(True)
        self.disconnect_action.setEnabled(False)
        self.status_label.setText("Disconnected")
        
        # Update console namespace
        self.console_namespace["thread"] = None
        self.console_namespace["motor"] = None
        self.setup_console_completion()
        
        print("Disconnected from serial port")
    
    def handle_connection_lost(self):
        QtWidgets.QMessageBox.warning(
            self,
            "Connection Lost",
            "Serial connection has been lost. Please reconnect."
        )
        self.disconnect_serial()

    def get_next_color(self):
        for color in self.available_colors:
            if color not in self.used_colors:
                self.used_colors.add(color)
                return color
        
        color = self.available_colors[self.color_index % len(self.available_colors)]
        self.color_index += 1
        return color
    
    def release_color(self, color):
        if color in self.used_colors:
            self.used_colors.remove(color)

    def on_data_received(self, values):
        if self.paused or not self.is_connected:
            return

        self.counter += 1
        self.time_buffer.append(self.counter)

        # remove channel
        for name in list(self.channels.keys()):
            if name not in self.motor.plotter_channels:
                ch = self.channels.pop(name)
                ch["line"].clear()
                self.plot_widget.removeItem(ch["line"])
                if "color" in ch:
                    self.release_color(ch["color"])
                if name in self.data_buffers:
                    self.data_buffers.remove(name)

        # add channel
        for idx, name in enumerate(self.motor.plotter_channels):
            if name not in self.channels:
                buffer = deque(
                    [np.nan] * (len(self.time_buffer)-1),
                    maxlen=self.max_points
                )
                self.data_buffers.append(buffer)
                
                color = self.get_next_color()
                
                pen = pg.mkPen(
                    color=color,
                    width=1.5
                )
                line = self.plot_widget.plot(
                    [],
                    [],
                    pen=pen,
                    name=name
                )
                self.channels[name] = {
                    "line": line,
                    "buffer": buffer,
                    "color": color
                }

        for idx, name in enumerate(self.motor.plotter_channels):
            if idx < len(values):
                self.channels[name]["buffer"].append(values[idx])
            else:
                self.channels[name]["buffer"].append(np.nan)

        self.status_label.setText(
            f"Received {len(values)} values"
        )
    
    def update_plot(self):
        if self.paused or not self.is_connected:
            return
        
        # Update setiap line
        x = np.asarray(self.time_buffer)
        for name in self.motor.plotter_channels:
            if name not in self.channels:
                continue
            ch = self.channels[name]
            y = np.asarray(ch["buffer"])
            n = min(len(x), len(y))
            if n == 0:
                continue
            ch["line"].setData(x[-n:], y[-n:])
        
        # Set X range
        if len(self.time_buffer) > 0:
            x_min = max(0, self.counter - self.max_points)
            x_max = self.counter
            self.plot_widget.setXRange(x_min, x_max, padding=0.05)

        # Auto-range jika diperlukan
        if self.auto_range_check.isChecked():
            self.auto_range()
        
        # Update info
        self.info_label.setText(f'Samples: {len(self.time_buffer)} | FPS: {self.fps_result}')
        self.fps_counter += 1
    
    def auto_range(self):
        if len(self.time_buffer) > 0:
            all_values = []
            
            for name in self.motor.plotter_channels:
                if name in self.channels:
                    buffer = self.channels[name]["buffer"]
                    all_values.extend(buffer)

            if not all_values:
                return
                
            all_values = np.asarray(all_values, dtype=float)
            all_values = all_values[~np.isnan(all_values)]

            if all_values.size == 0:
                return

            y_min = np.min(all_values)
            y_max = np.max(all_values)
            
            padding = max(1, (y_max - y_min) * 0.1)
            self.plot_widget.setYRange(
                y_min - padding,
                y_max + padding
            )
    
    def update_fps(self):
        self.fps_result = self.fps_counter
        self.fps_counter = 0
    
    def clear_data(self):
        for name in self.channels:
            self.channels[name]["buffer"].clear()
        self.time_buffer.clear()
        self.counter = 0
        
        for name in self.channels:
            self.channels[name]["line"].setData([], [])
        
        print("Data cleared")
    
    def toggle_pause(self):
        self.paused = not self.paused
        self.pause_button.setText('Resume' if self.paused else 'Pause')
        print(f"Plot {'paused' if self.paused else 'resumed'}")
    
    def closeEvent(self, event):
        print("Closing application...")
        self.disconnect_serial()
        
        if hasattr(self, 'timer'):
            self.timer.stop()
        if hasattr(self, 'fps_timer'):
            self.fps_timer.stop()
            
        event.accept()


def main():
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle('Fusion')
    
    pg.setConfigOptions(antialias=True, useOpenGL=True)
    
    plotter = LivePlotter(
        max_points=1000,
        port=None,  # No auto-connect
        baudrate=115200
    )
    
    plotter.show()
    
    # Run
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()