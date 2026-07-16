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

class DataAcquisitionThread(QtCore.QThread):
    """Thread untuk membaca data dari serial port"""
    
    data_received = QtCore.pyqtSignal(list)
    
    def __init__(self, num_channels=1, serial_conn=None, parent=None):
        super().__init__(parent)
        self.num_channels = num_channels
        self.running = True
        self.serial_conn = serial_conn
        
        # Queue untuk thread-safe data transfer
        self.data_queue = queue.Queue(maxsize=10000)
        
        # Buffer untuk data mentah
        self.raw_buffer = bytearray()
        
    def run(self):
        """Main loop untuk thread akuisisi data"""
        while self.running:
            try:
                # Baca data dari serial atau generate dummy
                if self.serial_conn: # and self.serial_conn.in_waiting:
                    # Baca data yang tersedia
                    raw_data = self.serial_conn.read(self.serial_conn.in_waiting)
                    if raw_data:
                        self.raw_buffer.extend(raw_data)
                elif not self.serial_conn:
                    # Simulasi data random
                    self.generate_dummy_data()
                
                # Parse data dari buffer
                self.parse_buffer()
                
                # Sedikit delay untuk menghindari CPU 100%
                QtCore.QThread.msleep(1)
                
            except Exception as e:
                print(f"Error di thread akuisisi: {e}")
                QtCore.QThread.msleep(10)
    
    def generate_dummy_data(self):
        import random
        import math

        t = time.time()
        values = []

        # jumlah channel random (1-8)
        num_data = random.randint(1, 8)

        for i in range(num_data):
            val = 5 * math.sin(2 * math.pi * 2 * t + i)
            val += random.uniform(-0.5, 0.5)
            values.append(val)

        frame = bytearray()

        frame.extend(struct.pack('<H', 0xABCD))
        frame.extend(struct.pack('<B', num_data))

        for val in values:
            frame.extend(struct.pack('<f', val))

        self.raw_buffer.extend(frame)
    
    def parse_buffer(self):
        HEADER_SIZE = 2
        SIZE_FIELD = 1

        while True:
            # minimal harus ada header + data_size
            if len(self.raw_buffer) < HEADER_SIZE + SIZE_FIELD:
                break
            header = struct.unpack('<H', self.raw_buffer[:2])[0]
            if header != 0xABCD:
                # geser 1 byte sampai ketemu header
                self.raw_buffer.pop(0)
                continue
            self.num_channels = self.raw_buffer[2]
            frame_size = (
                HEADER_SIZE +
                SIZE_FIELD +
                (self.num_channels * 4)
            )
            # tunggu sampai frame lengkap
            if len(self.raw_buffer) < frame_size:
                break
            try:
                values = []
                offset = 3
                for _ in range(self.num_channels):
                    value = struct.unpack(
                        '<f',
                        self.raw_buffer[offset:offset+4]
                    )[0]
                    values.append(value)
                    offset += 4
                self.data_received.emit(values)
                # hapus frame yang sudah diproses
                self.raw_buffer = self.raw_buffer[frame_size:]
            except Exception as e:
                print(f"Parse error: {e}")
                self.raw_buffer.pop(0)
    
    def stop(self):
        """Stop thread"""
        self.running = False
        if self.serial_conn:
            self.serial_conn.close()
        self.wait()


class LivePlotter(QtWidgets.QMainWindow):
    """Main window untuk live plotting"""
    
    def __init__(self, num_channels=4, max_points=1000, port=None, baudrate=115200):
        super().__init__()
        
        serial_conn = serial.Serial(
            port,
            baudrate,
            timeout=0.001
        )
        self.motor = MotorProtocol(serial_conn)

        self.num_channels = num_channels
        self.max_points = max_points
        
        # Setup UI
        self.setup_ui()
        
        # Buffer untuk data
        self.time_buffer = deque(maxlen=max_points)
        self.data_buffers = [deque(maxlen=max_points) for _ in range(num_channels)]
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
        
        # Setup performance timer
        self.last_update = time.time()
        self.fps_counter = 0
        self.fps_timer = QtCore.QTimer()
        self.fps_timer.timeout.connect(self.update_fps)
        self.fps_timer.start(1000)  # Update FPS setiap 1 detik
        
        # Status
        self.status_label = QtWidgets.QLabel("Ready")
        self.statusBar().addWidget(self.status_label)
        
        print(f"LivePlotter initialized dengan {num_channels} channel, {max_points} points")
    
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
        
        # Setup plot lines dengan warna berbeda
        colors = [
            (255, 0, 0),    # Red
            (0, 0, 255),    # Blue
            (0, 255, 0),    # Green
            (255, 165, 0),  # Orange
            (128, 0, 128),  # Purple
            (255, 192, 203),# Pink
            (0, 255, 255),  # Cyan
            (255, 0, 255)   # Magenta
        ]
        
        self.lines = []
        for i in range(self.num_channels):
            pen = pg.mkPen(color=colors[i % len(colors)], width=1.5)
            line = self.plot_widget.plot(
                [], [], 
                pen=pen, 
                name=f'CH{i+1}'
            )
            self.lines.append(line)
    
    def execute_command(self):
        cmd = self.console_input.text()
        self.console_output.appendPlainText(f">>> {cmd}")
        namespace = {
            "plotter": self,
            "thread": self.acq_thread,
            "motor": self.motor,
            "np": np,
            "pg": pg,
        }
        try:
            buffer = io.StringIO()
            with contextlib.redirect_stdout(buffer):
                try:
                    result = eval(cmd, globals(), namespace)
                    if result is not None:
                        print(result)
                except SyntaxError:
                    exec(cmd, globals(), namespace)
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
        # Tambah channel jika diperlukan
        while len(self.data_buffers) < len(values):
            idx = len(self.data_buffers)
            self.data_buffers.append(
                deque(maxlen=self.max_points)
            )
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
            if len(self.data_buffers[i]) > 0:
                # Konversi ke numpy array untuk performa
                x_data = np.array(list(self.time_buffer))
                y_data = np.array(list(self.data_buffers[i]))
                line.setData(x_data, y_data)
        
        # Auto-range jika diperlukan
        if self.auto_range_check.isChecked():
            self.auto_range()
        
        # Update info
        self.info_label.setText(f'Samples: {len(self.time_buffer)} | FPS: {self.fps_counter}')
        self.fps_counter += 1
    
    def auto_range(self):
        """Auto scale plot"""
        if len(self.time_buffer) > 0:
            # Set X range
            x_min = max(0, self.counter - self.max_points)
            x_max = self.counter
            self.plot_widget.setXRange(x_min, x_max, padding=0.05)
            
            # Set Y range berdasarkan semua channel
            all_values = []
            for buffer in self.data_buffers:
                if len(buffer) > 0:
                    all_values.extend(buffer)
            
            if all_values:
                y_min = min(all_values)
                y_max = max(all_values)
                y_range = y_max - y_min
                if y_range > 0.001:
                    self.plot_widget.setYRange(
                        y_min - y_range * 0.1,
                        y_max + y_range * 0.1
                    )
                else:
                    self.plot_widget.setYRange(y_min - 1, y_max + 1)
    
    def update_fps(self):
        """Update FPS counter"""
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
    # Ganti parameter sesuai kebutuhan:
    # - num_channels: jumlah channel data
    # - max_points: jumlah titik yang ditampilkan
    # - port: port serial (None untuk simulasi)
    # - baudrate: baudrate serial
    
    plotter = LivePlotter(
        num_channels=1,
        max_points=1000,
        port="COM4",  # Ganti dengan 'COM3' atau '/dev/ttyUSB0' untuk data real
        baudrate=115200
    )
    
    plotter.show()
    
    # Run
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()