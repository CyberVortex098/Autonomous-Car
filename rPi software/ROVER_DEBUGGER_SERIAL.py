import sys
import collections
import serial
import serial.tools.list_ports
from PyQt5 import QtWidgets, QtCore
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

# Maximum data points to retain on the plot buffer
MAX_HISTORY = 300

# Mapping message types to their respective CSV fields
FIELD_MAP = {
    "$IMU":    ["Gyro X", "Gyro Y", "Gyro Z", "Accel X", "Accel Y", "Accel Z", "Roll", "Pitch", "Yaw"],
    "$DIST":   ["Dist 0 (mm)", "Dist 1 (mm)", "Dist 2 (mm)", "Dist 3 (mm)"],
    "$COLOR":  ["Lux", "Red %", "Green %", "Blue %"],
    "$ADC":    ["ADC 0 (%)", "ADC 1 (%)", "ADC 2 (%)"],
    "$BUTTON": ["Button 1", "Button 2"]
}

class SerialHandler(QtCore.QThread):
    """Thread handling non-blocking serial read & write operations."""
    line_received = QtCore.pyqtSignal(str)

    def __init__(self, port, baudrate=921600):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.running = True
        self.ser = None

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            while self.running:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.line_received.emit(line)
            self.ser.close()
        except Exception as e:
            print(f"Serial Error: {e}")

    def send_command(self, cmd: str):
        """Send command string over serial line."""
        if self.ser and self.ser.is_open:
            full_cmd = (cmd.strip() + "\n").encode('utf-8')
            self.ser.write(full_cmd)

    def stop(self):
        self.running = False
        self.wait()

class MplCanvas(FigureCanvas):
    """Matplotlib Canvas Widget embedded into PyQt5."""
    def __init__(self, parent=None, width=8, height=6, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.ax = self.fig.add_subplot(111)
        super().__init__(self.fig)
        self.ax.set_title("Real-Time Telemetry Plot")
        self.ax.set_xlabel("Samples")
        self.ax.set_ylabel("Value")
        self.ax.grid(True)

class SerialPlotterApp(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Embedded Telemetry & Control Dashboard")
        self.resize(1200, 800)

        # Storage buffers for time-series data
        self.data_buffers = {}
        self.checkboxes = {}

        self.serial_thread = None
        self.led1_state = False
        self.led2_state = False

        self.init_buffers()
        self.init_ui()

        # Update plot timer (30 FPS refresh rate)
        self.plot_timer = QtCore.QTimer()
        self.plot_timer.setInterval(33)
        self.plot_timer.timeout.connect(self.update_plot)
        self.plot_timer.start()

    def init_buffers(self):
        """Initialize data queues for every variable field."""
        for msg, fields in FIELD_MAP.items():
            self.data_buffers[msg] = {}
            for field in fields:
                self.data_buffers[msg][field] = collections.deque(maxlen=MAX_HISTORY)

    def init_ui(self):
        main_widget = QtWidgets.QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QtWidgets.QVBoxLayout(main_widget)

        # --- Top Connection Bar ---
        top_bar = QtWidgets.QHBoxLayout()
        top_bar.addWidget(QtWidgets.QLabel("Serial Port:"))
        
        self.port_combo = QtWidgets.QComboBox()
        self.refresh_ports()
        top_bar.addWidget(self.port_combo)

        self.btn_refresh = QtWidgets.QPushButton("Refresh Ports")
        self.btn_refresh.clicked.connect(self.refresh_ports)
        top_bar.addWidget(self.btn_refresh)

        self.btn_connect = QtWidgets.QPushButton("Connect")
        self.btn_connect.clicked.connect(self.toggle_connection)
        top_bar.addWidget(self.btn_connect)
        
        top_bar.addStretch()
        main_layout.addLayout(top_bar)

        # --- Tab Widget ---
        self.tabs = QtWidgets.QTabWidget()
        main_layout.addWidget(self.tabs)

        # Tab 1: Telemetry Plotter
        tab_telemetry = QtWidgets.QWidget()
        self.init_telemetry_tab(tab_telemetry)
        self.tabs.addTab(tab_telemetry, "Telemetry Plotter")

        # Tab 2: Outputs Control
        tab_outputs = QtWidgets.QWidget()
        self.init_outputs_tab(tab_outputs)
        self.tabs.addTab(tab_outputs, "Outputs")

    # ================= Telemetry Tab =================
    def init_telemetry_tab(self, parent):
        layout = QtWidgets.QHBoxLayout(parent)

        # Left Control Panel
        control_panel = QtWidgets.QWidget()
        control_layout = QtWidgets.QVBoxLayout(control_panel)
        control_panel.setMaximumWidth(320)

        sel_group = QtWidgets.QGroupBox("Plot Selectors")
        sel_layout = QtWidgets.QVBoxLayout(sel_group)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_content = QtWidgets.QWidget()
        scroll_layout = QtWidgets.QVBoxLayout(scroll_content)

        for msg, fields in FIELD_MAP.items():
            msg_box = QtWidgets.QGroupBox(msg)
            msg_box_layout = QtWidgets.QVBoxLayout(msg_box)
            self.checkboxes[msg] = {}

            for field in fields:
                cb = QtWidgets.QCheckBox(field)
                cb.setChecked(False)
                self.checkboxes[msg][field] = cb
                msg_box_layout.addWidget(cb)

            scroll_layout.addWidget(msg_box)

        scroll_content.setLayout(scroll_layout)
        scroll.setWidget(scroll_content)
        sel_layout.addWidget(scroll)
        control_layout.addWidget(sel_group)

        btn_clear = QtWidgets.QPushButton("Clear Buffers")
        btn_clear.clicked.connect(self.clear_buffers)
        control_layout.addWidget(btn_clear)

        # Right Plot Area
        self.canvas = MplCanvas(self)

        layout.addWidget(control_panel)
        layout.addWidget(self.canvas)

    # ================= Outputs Tab =================
    def init_outputs_tab(self, parent):
        layout = QtWidgets.QVBoxLayout(parent)

        # 1. LED Controls Section
        led_group = QtWidgets.QGroupBox("LED Control")
        led_layout = QtWidgets.QHBoxLayout(led_group)

        self.btn_led1_on = QtWidgets.QPushButton("LED 1 ON")
        self.btn_led1_off = QtWidgets.QPushButton("LED 1 OFF")
        self.btn_led2_on = QtWidgets.QPushButton("LED 2 ON")
        self.btn_led2_off = QtWidgets.QPushButton("LED 2 OFF")

        self.btn_led1_on.clicked.connect(lambda: self.set_led(1, True))
        self.btn_led1_off.clicked.connect(lambda: self.set_led(1, False))
        self.btn_led2_on.clicked.connect(lambda: self.set_led(2, True))
        self.btn_led2_off.clicked.connect(lambda: self.set_led(2, False))

        led_layout.addWidget(self.btn_led1_on)
        led_layout.addWidget(self.btn_led1_off)
        led_layout.addSpacing(20)
        led_layout.addWidget(self.btn_led2_on)
        led_layout.addWidget(self.btn_led2_off)
        led_layout.addStretch()

        layout.addWidget(led_group)

        # 2. Buzzer Control Section
        buzzer_group = QtWidgets.QGroupBox("Buzzer Control ($BEEP)")
        buzzer_layout = QtWidgets.QHBoxLayout(buzzer_group)

        # Frequency input
        buzzer_layout.addWidget(QtWidgets.QLabel("Frequency (Hz):"))
        self.spin_freq = QtWidgets.QSpinBox()
        self.spin_freq.setRange(50, 10000)
        self.spin_freq.setValue(1000)
        self.spin_freq.setSingleStep(100)
        buzzer_layout.addWidget(self.spin_freq)

        # Duration input
        buzzer_layout.addWidget(QtWidgets.QLabel("Duration (ms):"))
        self.spin_dur = QtWidgets.QSpinBox()
        self.spin_dur.setRange(10, 5000)
        self.spin_dur.setValue(200)
        self.spin_dur.setSingleStep(50)
        buzzer_layout.addWidget(self.spin_dur)

        # Play Button
        btn_play_beep = QtWidgets.QPushButton("Trigger Beep")
        btn_play_beep.clicked.connect(self.send_beep_cmd)
        buzzer_layout.addWidget(btn_play_beep)

        # Preset Tone Buttons
        btn_preset_alert = QtWidgets.QPushButton("Preset: Alert (2kHz / 100ms)")
        btn_preset_alert.clicked.connect(lambda: self.send_custom_beep(2000, 100))
        buzzer_layout.addWidget(btn_preset_alert)

        btn_preset_error = QtWidgets.QPushButton("Preset: Low Warning (400Hz / 400ms)")
        btn_preset_error.clicked.connect(lambda: self.send_custom_beep(400, 400))
        buzzer_layout.addWidget(btn_preset_error)

        buzzer_layout.addStretch()
        layout.addWidget(buzzer_group)

        # 3. Servo Controls Section (16 Channels)
        servo_group = QtWidgets.QGroupBox("PCA9685 Servos (0 - 180°)")
        servo_main_layout = QtWidgets.QVBoxLayout(servo_group)

        scroll_servo = QtWidgets.QScrollArea()
        scroll_servo.setWidgetResizable(True)
        servo_content = QtWidgets.QWidget()
        servo_grid = QtWidgets.QGridLayout(servo_content)

        self.servo_sliders = []
        self.servo_labels = []

        for ch in range(16):
            row = ch // 2
            col_offset = (ch % 2) * 4

            lbl_title = QtWidgets.QLabel(f"<b>Ch {ch}:</b>")
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            slider.setRange(0, 180)
            slider.setValue(90)

            lbl_val = QtWidgets.QLabel("90°")
            lbl_val.setMinimumWidth(35)
            
            slider.valueChanged.connect(lambda val, l=lbl_val: l.setText(f"{val}°"))

            btn_update = QtWidgets.QPushButton("Update")
            btn_update.clicked.connect(lambda _, c=ch, s=slider: self.send_servo_cmd(c, s.value()))

            servo_grid.addWidget(lbl_title, row, col_offset)
            servo_grid.addWidget(slider, row, col_offset + 1)
            servo_grid.addWidget(lbl_val, row, col_offset + 2)
            servo_grid.addWidget(btn_update, row, col_offset + 3)

            self.servo_sliders.append(slider)
            self.servo_labels.append(lbl_val)

        servo_content.setLayout(servo_grid)
        scroll_servo.setWidget(servo_content)
        servo_main_layout.addWidget(scroll_servo)

        # Batch Servo Action Bar
        batch_layout = QtWidgets.QHBoxLayout()
        btn_all_90 = QtWidgets.QPushButton("Set All to 90°")
        btn_all_90.clicked.connect(self.set_all_servos_90)
        batch_layout.addWidget(btn_all_90)
        batch_layout.addStretch()

        servo_main_layout.addLayout(batch_layout)
        layout.addWidget(servo_group)

    # ================= Command Handlers =================
    def set_led(self, led_num, state):
        if led_num == 1:
            self.led1_state = state
        elif led_num == 2:
            self.led2_state = state

        cmd = f"$LED,{1 if self.led1_state else 0},{1 if self.led2_state else 0}"
        self.send_command(cmd)

    def send_beep_cmd(self):
        freq = self.spin_freq.value()
        dur = self.spin_dur.value()
        cmd = f"$BEEP,{freq},{dur}"
        self.send_command(cmd)

    def send_custom_beep(self, freq, dur):
        cmd = f"$BEEP,{freq},{dur}"
        self.send_command(cmd)

    def send_servo_cmd(self, channel, angle):
        cmd = f"$SERVO,{channel},{angle},False,15"
        self.send_command(cmd)

    def set_all_servos_90(self):
        for ch, slider in enumerate(self.servo_sliders):
            slider.setValue(90)
            self.send_servo_cmd(ch, 90)

    def send_command(self, cmd):
        if self.serial_thread and self.serial_thread.isRunning():
            self.serial_thread.send_command(cmd)
            print(f"Sent: {cmd}")
        else:
            print(f"Not Connected. Command dropped: {cmd}")

    # ================= Serial & UI Helpers =================
    def refresh_ports(self):
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for p in ports:
            self.port_combo.addItem(p.device)

    def toggle_connection(self):
        if self.serial_thread and self.serial_thread.isRunning():
            self.serial_thread.stop()
            self.serial_thread = None
            self.btn_connect.setText("Connect")
            self.port_combo.setEnabled(True)
        else:
            port = self.port_combo.currentText()
            if not port:
                return
            self.serial_thread = SerialHandler(port, baudrate=921600)
            self.serial_thread.line_received.connect(self.parse_line)
            self.serial_thread.start()
            self.btn_connect.setText("Disconnect")
            self.port_combo.setEnabled(False)

    def parse_line(self, line):
        tokens = line.split(',')
        if not tokens:
            return

        header = tokens[0]
        if header in FIELD_MAP:
            fields = FIELD_MAP[header]
            for i, field in enumerate(fields):
                if i + 1 < len(tokens):
                    try:
                        val = float(tokens[i + 1])
                        self.data_buffers[header][field].append(val)
                    except ValueError:
                        pass

    def clear_buffers(self):
        for msg in self.data_buffers:
            for field in self.data_buffers[msg]:
                self.data_buffers[msg][field].clear()

    def update_plot(self):
        if self.tabs.currentIndex() != 0:
            return

        self.canvas.ax.cla()
        self.canvas.ax.grid(True)
        self.canvas.ax.set_title("Real-Time Telemetry Plot")
        self.canvas.ax.set_xlabel("Samples")
        self.canvas.ax.set_ylabel("Value")

        plotted = False
        for msg, fields in self.checkboxes.items():
            for field, cb in fields.items():
                if cb.isChecked():
                    data = list(self.data_buffers[msg][field])
                    if data:
                        self.canvas.ax.plot(data, label=f"{msg} -> {field}")
                        plotted = True

        if plotted:
            self.canvas.ax.legend(loc="upper left")

        self.canvas.draw()

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = SerialPlotterApp()
    window.show()
    sys.exit(app.exec_())
