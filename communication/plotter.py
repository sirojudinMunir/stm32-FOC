import sys
import struct
import numpy as np
import pyqtgraph as pg
from PyQt5 import QtCore, QtWidgets
from collections import deque
import serial
import threading
import queue
import time
import io
import contextlib
import traceback
from motor_protocol import MotorProtocol
import queue

class DataAcquisitionThread(QtCore.QThread):
    """Thread untuk membaca data dari serial port"""
    
    data_received = QtCore.pyqtSignal(list)
    
    def __init__(self, serial_conn=None, parent=None):
        super().__init__(parent)
        self.running = True
        self.serial_conn = serial_conn
        self.data_queue = queue.Queue(maxsize=10000)
        self.raw_buffer = bytearray()
        self.response_queue = queue.Queue()
        self.expected_response_size = None
        
    def run(self):
        """Main loop untuk thread akuisisi data"""
        while self.running:
            try:
                if self.serial_conn:
                    raw_data = self.serial_conn.read(self.serial_conn.in_waiting)
                    if raw_data:
                        self.raw_buffer.extend(raw_data)
                
                self.parse_buffer()
                QtCore.QThread.msleep(1)
                
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
        """Stop thread"""
        self.running = False
        if self.serial_conn:
            self.serial_conn.close()
        self.wait()


class LivePlotter(QtWidgets.QMainWindow):
    """Main window untuk live plotting"""
    
    def __init__(self, max_points=1000, port=None, baudrate=115200):
        super().__init__()
        
        serial_conn = serial.Serial(
            port,
            baudrate,
            timeout=0.001
        )

        self.max_points = max_points
        
        # Setup UI
        self.setup_ui()
        
        # Buffer untuk data
        self.time_buffer = deque(maxlen=max_points)
        self.data_buffers = []
        self.counter = 0
        
        # Timer untuk update plot
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(10)  # Update setiap 10ms (100 FPS)
        
        # Start acquisition thread
        self.acq_thread = DataAcquisitionThread(
            serial_conn=serial_conn
        )
        self.acq_thread.data_received.connect(self.on_data_received)
        self.acq_thread.start()
        
        self.motor = MotorProtocol(
            serial_conn,
            self.acq_thread
        )
        
        # Setup performance timer
        self.last_update = time.time()
        self.fps_counter = 0
        self.fps_result = 0
        self.fps_timer = QtCore.QTimer()
        self.fps_timer.timeout.connect(self.update_fps)
        self.fps_timer.start(1000)  # Update FPS setiap 1 detik
        
        # Status
        self.status_label = QtWidgets.QLabel("Ready")
        self.statusBar().addWidget(self.status_label)
        
        self.console_namespace = {
            "plotter": self,
            "thread": self.acq_thread,
            "motor": self.motor,
            "np": np,
            "pg": pg,
        }

        self.setup_console_completion()

        print(f"LivePlotter initialized")
    
    def setup_ui(self):
        """Setup user interface"""
        self.setWindowTitle('Live Plotter - PyQtGraph')
        self.setGeometry(100, 100, 1200, 600)
        
        # Central widget
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        layout = QtWidgets.QVBoxLayout(central_widget)
        
        # Toolbar
        toolbar = QtWidgets.QToolBar()
        self.addToolBar(toolbar)
        
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
        
        # Setup plot lines
        # colors = [
        #     (255, 0, 0),    # Red
        #     (0, 0, 255),    # Blue
        #     (0, 255, 0),    # Green
        #     (255, 165, 0),  # Orange
        #     (128, 0, 128),  # Purple
        #     (255, 192, 203),# Pink
        #     (0, 255, 255),  # Cyan
        #     (255, 0, 255)   # Magenta
        # ]
        
        self.lines = []
        # for i in range(self.num_channels):
        #     pen = pg.mkPen(color=colors[i % len(colors)], width=1.5)
        #     line = self.plot_widget.plot(
        #         [], [], 
        #         pen=pen, 
        #         name=f'CH{i+1}'
        #     )
        #     self.lines.append(line)

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

    def on_data_received(self, values):
        if self.paused:
            return
        self.counter += 1
        self.time_buffer.append(self.counter)
        colors = [
            (255, 0, 0),      # Red
            (0, 0, 255),      # Blue
            (0, 255, 0),      # Green
            (255, 165, 0),    # Orange
            (128, 0, 128),    # Purple
            (255, 192, 203),  # Pink
            (0, 255, 255),    # Cyan
            (255, 0, 255)     # Magenta
        ]

        while len(self.data_buffers) > len(values):
            self.data_buffers.pop()
            line = self.lines.pop()
            line.clear()
            self.plot_widget.removeItem(line)

        # Tambah channel jika diperlukan
        while len(self.data_buffers) < len(values):
            idx = len(self.data_buffers)
            buffer = deque(maxlen=self.max_points)
            if len(self.time_buffer) > 1:
                buffer.extend(
                    [np.nan] * (len(self.time_buffer)-1)
                )
            self.data_buffers.append(buffer)
            pen = pg.mkPen(
                color=colors[idx % len(colors)],
                width=1.5
            )
            line = self.plot_widget.plot(
                [],
                [],
                pen=pen,
                name=f"CH{idx+1}"
            )
            self.lines.append(line)
        # Simpan data
        for i, val in enumerate(values):
            self.data_buffers[i].append(val)
        self.status_label.setText(
            f"Received {len(values)} values"
        )
    
    def update_plot(self):
        """Update plot dengan data terbaru"""
        if self.paused:
            return
        
        # Cek apakah ada data baru
        has_data = any(len(buf) > 0 for buf in self.data_buffers)
        if not has_data:
            return
        
        # Update setiap line
        for i, line in enumerate(self.lines):
            # if len(self.data_buffers[i]) > 0:
            # Konversi ke numpy array untuk performa
            x_data = np.array(list(self.time_buffer))
            y_data = np.array(list(self.data_buffers[i]))
            line.setData(x_data, y_data)
        
        # Set X range
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
        """Auto scale plot"""
        if len(self.time_buffer) > 0:
            # Set Y range berdasarkan semua channel
            all_values = []

            for buf in self.data_buffers:
                all_values.extend(buf)

            all_values = np.asarray(all_values, dtype=float)

            # Buang NaN
            all_values = all_values[~np.isnan(all_values)]

            # Tidak ada data valid
            if all_values.size == 0:
                return

            y_min = np.min(all_values)
            y_max = np.max(all_values)

            self.plot_widget.setYRange(
                y_min - 1,
                y_max + 1
            )
    
    def update_fps(self):
        """Update FPS counter"""
        self.fps_result = self.fps_counter
        self.fps_counter = 0
    
    def clear_data(self):
        """Clear all data"""
        for buffer in self.data_buffers:
            buffer.clear()
        self.time_buffer.clear()
        self.counter = 0
        
        for line in self.lines:
            line.setData([], [])
        
        print("Data cleared")
    
    def toggle_pause(self):
        """Toggle pause/resume"""
        self.paused = not self.paused
        self.pause_button.setText('Resume' if self.paused else 'Pause')
        print(f"Plot {'paused' if self.paused else 'resumed'}")
    
    def closeEvent(self, event):
        """Handle window close"""
        print("Closing application...")
        self.acq_thread.stop()
        event.accept()


def main():
    """Main function"""
    # Setup Qt application
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle('Fusion')
    
    # Setup PyQtGraph
    pg.setConfigOptions(antialias=True, useOpenGL=True)  # Enable OpenGL for speed
    
    # Buat plotter
    plotter = LivePlotter(
        max_points=1000,
        port="COM4",  # Ganti dengan 'COM3' atau '/dev/ttyUSB0' untuk data real
        baudrate=115200
    )
    
    plotter.show()
    
    # Run
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()