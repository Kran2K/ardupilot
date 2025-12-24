import tkinter as tk
from tkinter import ttk, scrolledtext
import serial.tools.list_ports
from pymavlink import mavutil
import threading
import time
import queue
import math
import datetime

# --- Tooltip Helper Class ---
class CreateToolTip:
    def __init__(self, widget, text):
        self.widget = widget
        self.text = text
        self.tooltip_window = None
        self.id = None
        self.widget.bind("<Enter>", self.enter)
        self.widget.bind("<Leave>", self.leave)

    def enter(self, event=None):
        self.id = self.widget.after(150, self.show_tooltip)

    def leave(self, event=None):
        if self.id:
            self.widget.after_cancel(self.id)
            self.id = None
        self.hide_tooltip()

    def show_tooltip(self):
        if self.tooltip_window or not self.text:
            return
        x, y, _, _ = self.widget.bbox("insert")
        x += self.widget.winfo_rootx() + 25
        y += self.widget.winfo_rooty() + 25
        
        self.tooltip_window = tw = tk.Toplevel(self.widget)
        tw.wm_overrideredirect(True)
        tw.wm_geometry(f"+{x}+{y}")
        
        label = tk.Label(tw, text=self.text, justify=tk.LEFT,
                         background="#ffffe0", relief=tk.SOLID, borderwidth=1,
                         font=("tahoma", "8", "normal"), wraplength=400)
        label.pack(ipadx=1)

    def hide_tooltip(self):
        if self.tooltip_window:
            self.tooltip_window.destroy()
        self.tooltip_window = None

class AdvancedMAVLinkHILGUI:
    MAVLINK_MESSAGE_INFO = {
        'HIL_SENSOR': "The IMU readings in SI units in NED body frame.",
        'GPS_INPUT': "GPS sensor input message. This is a raw sensor value sent by the GPS. This is NOT the global position estimate of the system.",
        'HIL_STATE_QUATERNION': "Sent from simulation to autopilot, avoids in contrast to HIL_STATE singularities. This packet is useful for high throughput applications such as hardware in the loop simulations."
    }

    MAVLINK_FIELD_INFO = {
        # HIL_SENSOR (107)
        'xacc': {'desc': 'X acceleration', 'unit': 'm/s/s', 'type': 'float'},
        'yacc': {'desc': 'Y acceleration', 'unit': 'm/s/s', 'type': 'float'},
        'zacc': {'desc': 'Z acceleration', 'unit': 'm/s/s', 'type': 'float'},
        'xgyro': {'desc': 'Angular speed around X axis in body frame', 'unit': 'rad/s', 'type': 'float'},
        'ygyro': {'desc': 'Angular speed around Y axis in body frame', 'unit': 'rad/s', 'type': 'float'},
        'zgyro': {'desc': 'Angular speed around Z axis in body frame', 'unit': 'rad/s', 'type': 'float'},
        'xmag': {'desc': 'X Magnetic field', 'unit': 'gauss', 'type': 'float'},
        'ymag': {'desc': 'Y Magnetic field', 'unit': 'gauss', 'type': 'float'},
        'zmag': {'desc': 'Z Magnetic field', 'unit': 'gauss', 'type': 'float'},
        'abs_pressure': {'desc': 'Absolute pressure', 'unit': 'hPa', 'type': 'float'},
        'diff_pressure': {'desc': 'Differential pressure (airspeed)', 'unit': 'hPa', 'type': 'float'},
        'pressure_alt': {'desc': 'Altitude calculated from pressure', 'unit': '', 'type': 'float'},
        'temperature': {'desc': 'Temperature', 'unit': 'degC', 'type': 'float'},
        'fields_updated': {'desc': 'Bitmap for fields that have updated since last message', 'unit': 'HIL_SENSOR_UPDATED_FLAGS', 'type': 'uint32_t'},

        # GPS_INPUT (232)
        'time_usec': {'desc': 'Timestamp (UNIX Epoch time or time since system boot)', 'unit': 'us', 'type': 'uint64_t'},
        'gps_id': {'desc': 'ID of the GPS for multiple GPS inputs', 'unit': '', 'type': 'uint8_t'},
        'ignore_flags': {'desc': 'Bitmap indicating which GPS input flags fields to ignore', 'unit': 'GPS_INPUT_IGNORE_FLAGS', 'type': 'uint16_t'},
        'time_week_ms': {'desc': 'GPS time (from start of GPS week)', 'unit': 'ms', 'type': 'uint32_t'},
        'time_week': {'desc': 'GPS week number', 'unit': '', 'type': 'uint16_t'},
        'fix_type': {'desc': '0-1: no fix, 2: 2D fix, 3: 3D fix, 4: 3D with DGPS, 5: 3D with RTK', 'unit': '', 'type': 'uint8_t'},
        'lat': {'desc': 'Latitude (WGS84)', 'unit': 'degE7', 'type': 'int32_t'},
        'lon': {'desc': 'Longitude (WGS84)', 'unit': 'degE7', 'type': 'int32_t'},
        'alt': {'desc': 'Altitude (MSL). Positive for up', 'unit': 'm', 'type': 'float'},
        'hdop': {'desc': 'GPS HDOP horizontal dilution of position (unitless)', 'unit': '', 'type': 'float'},
        'vdop': {'desc': 'GPS VDOP vertical dilution of position (unitless)', 'unit': '', 'type': 'float'},
        'vn': {'desc': 'GPS velocity in north direction in earth-fixed NED frame', 'unit': 'm/s', 'type': 'float'},
        've': {'desc': 'GPS velocity in east direction in earth-fixed NED frame', 'unit': 'm/s', 'type': 'float'},
        'vd': {'desc': 'GPS velocity in down direction in earth-fixed NED frame', 'unit': 'm/s', 'type': 'float'},
        'speed_accuracy': {'desc': 'GPS speed accuracy', 'unit': 'm/s', 'type': 'float'},
        'horiz_accuracy': {'desc': 'GPS horizontal accuracy', 'unit': 'm', 'type': 'float'},
        'vert_accuracy': {'desc': 'GPS vertical accuracy', 'unit': 'm', 'type': 'float'},
        'satellites_visible': {'desc': 'Number of satellites visible', 'unit': '', 'type': 'uint8_t'},
        'yaw': {'desc': "Yaw of vehicle relative to Earth's North, zero means not available, use 36000 for north", 'unit': 'cdeg', 'type': 'uint16_t'},

        # HIL_STATE_QUATERNION (115)
        'q1': {'desc': 'Vehicle attitude as normalized quaternion (w component)', 'unit': '', 'type': 'float[4]'},
        'q2': {'desc': 'Vehicle attitude as normalized quaternion (x component)', 'unit': '', 'type': 'float[4]'},
        'q3': {'desc': 'Vehicle attitude as normalized quaternion (y component)', 'unit': '', 'type': 'float[4]'},
        'q4': {'desc': 'Vehicle attitude as normalized quaternion (z component)', 'unit': '', 'type': 'float[4]'},
        'rollspeed': {'desc': 'Body frame roll / phi angular speed', 'unit': 'rad/s', 'type': 'float'},
        'pitchspeed': {'desc': 'Body frame pitch / theta angular speed', 'unit': 'rad/s', 'type': 'float'},
        'yawspeed': {'desc': 'Body frame yaw / psi angular speed', 'unit': 'rad/s', 'type': 'float'},
        'vx': {'desc': 'Ground X Speed (Latitude)', 'unit': 'cm/s', 'type': 'int16_t'},
        'vy': {'desc': 'Ground Y Speed (Longitude)', 'unit': 'cm/s', 'type': 'int16_t'},
        'vz': {'desc': 'Ground Z Speed (Altitude)', 'unit': 'cm/s', 'type': 'int16_t'},
        'ind_airspeed': {'desc': 'Indicated airspeed', 'unit': 'cm/s', 'type': 'uint16_t'},
        'true_airspeed': {'desc': 'True airspeed', 'unit': 'cm/s', 'type': 'uint16_t'},
        
        # HIL_STATE_QUATERNION 필드 중복 처리
        'HIL_STATE_QUATERNION_lat': {'desc': 'Latitude', 'unit': 'degE7', 'type': 'int32_t'},
        'HIL_STATE_QUATERNION_lon': {'desc': 'Longitude', 'unit': 'degE7', 'type': 'int32_t'},
        'HIL_STATE_QUATERNION_alt': {'desc': 'Altitude', 'unit': 'mm', 'type': 'int32_t'},
        'HIL_STATE_QUATERNION_xacc': {'desc': 'X acceleration', 'unit': 'mG', 'type': 'int16_t'},
        'HIL_STATE_QUATERNION_yacc': {'desc': 'Y acceleration', 'unit': 'mG', 'type': 'int16_t'},
        'HIL_STATE_QUATERNION_zacc': {'desc': 'Z acceleration', 'unit': 'mG', 'type': 'int16_t'},
    }

    def __init__(self, master):
        self.master = master
        self.master.title("Tandem Mavlink HIL Tester")
        self.master.geometry("880x880")

        self.mav_connection = None
        self.is_running = threading.Event()
        self.periodic_senders = {}
        self.gui_queue = queue.Queue()
        self.MAX_LOG_LINES = 500

        self._create_widgets()
        self.master.protocol("WM_DELETE_WINDOW", self._on_closing)
        self.master.after(100, self._process_gui_queue)

    def _create_widgets(self):
        main_frame = ttk.Frame(self.master, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        top_frame = ttk.Frame(main_frame)
        top_frame.pack(fill=tk.X, expand=False, pady=5)

        conn_frame = ttk.LabelFrame(top_frame, text="연결 설정", padding="10")
        conn_frame.pack(side=tk.TOP, fill=tk.X, expand=True, pady=(0, 5))
        self._create_connection_widgets(conn_frame)

        status_frame = ttk.LabelFrame(top_frame, text="실시간 상태", padding="10")
        status_frame.pack(side=tk.TOP, fill=tk.X, expand=True)
        self._create_status_display(status_frame)
        
        sender_frame = ttk.LabelFrame(main_frame, text="입력 (브릿지 → ArduPilot)", padding="10")
        sender_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        self._create_sender_widgets(sender_frame)

        log_frame = ttk.Frame(main_frame)
        log_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        self._create_log_widgets(log_frame)

    def _create_connection_widgets(self, parent):
        parent.columnconfigure(1, weight=1)
        ttk.Label(parent, text="COM 포트:").grid(row=0, column=0, sticky="w", padx=5, pady=2)
        self.com_port_combo = ttk.Combobox(parent, values=self._get_com_ports())
        self.com_port_combo.grid(row=0, column=1, sticky="ew", padx=5, pady=2)
        if self.com_port_combo['values']:
            self.com_port_combo.current(0)
        
        refresh_btn = ttk.Button(parent, text="갱신", command=lambda: self.com_port_combo.config(values=self._get_com_ports()))
        refresh_btn.grid(row=0, column=2, padx=(5,0))

        ttk.Label(parent, text="Baudrate:").grid(row=1, column=0, sticky="w", padx=5, pady=2)
        baud_rates = ['9600', '19200', '38400', '57600', '115200', '230400', '460800', '921600']
        self.baud_rate_combo = ttk.Combobox(parent, values=baud_rates)
        self.baud_rate_combo.grid(row=1, column=1, sticky="ew", padx=5, pady=2)
        self.baud_rate_combo.set('115200')

        self.connect_btn = ttk.Button(parent, text="연결", command=self._toggle_connection)
        self.connect_btn.grid(row=1, column=2, padx=(5,0), sticky='ew')

    def _create_status_display(self, parent):
        hb_frame = ttk.LabelFrame(parent, text="Heartbeat Status")
        hb_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))

        self.hb_vars = {
            'Mode': tk.StringVar(value='N/A'), 'System Status': tk.StringVar(value='N/A'),
            'Type': tk.StringVar(value='N/A'), 'Autopilot': tk.StringVar(value='N/A')
        }
        ttk.Label(hb_frame, text="Type:", font=('Helvetica', 9, 'bold')).grid(row=0, column=0, sticky="w")
        ttk.Label(hb_frame, textvariable=self.hb_vars['Type']).grid(row=0, column=1, sticky="w", padx=(0,10))
        ttk.Label(hb_frame, text="Autopilot:", font=('Helvetica', 9, 'bold')).grid(row=0, column=2, sticky="w")
        ttk.Label(hb_frame, textvariable=self.hb_vars['Autopilot']).grid(row=0, column=3, sticky="w", padx=(0,10))
        ttk.Label(hb_frame, text="Mode:", font=('Helvetica', 9, 'bold')).grid(row=1, column=0, sticky="w")
        ttk.Label(hb_frame, textvariable=self.hb_vars['Mode']).grid(row=1, column=1, sticky="w", padx=(0,10))
        ttk.Label(hb_frame, text="Status:", font=('Helvetica', 9, 'bold')).grid(row=1, column=2, sticky="w")
        ttk.Label(hb_frame, textvariable=self.hb_vars['System Status']).grid(row=1, column=3, sticky="w", padx=(0,10))
        
        servo_frame = ttk.LabelFrame(parent, text="Servo Outputs (1-8)")
        servo_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(5, 0))
        self.servo_vars = [tk.StringVar(value='N/A') for _ in range(8)]
        for i in range(8):
            row, col = i // 4, i % 4
            ttk.Label(servo_frame, text=f"{i+1}:").grid(row=row, column=2*col, sticky="w", padx=2, pady=1)
            ttk.Label(servo_frame, textvariable=self.servo_vars[i], width=6).grid(row=row, column=2*col+1, sticky="w", padx=2, pady=1)

    def _create_sender_widgets(self, parent):
        self.message_entries = {}
        notebook = ttk.Notebook(parent)
        notebook.pack(fill=tk.BOTH, expand=True)

        tab1 = ttk.Frame(notebook)
        tab2 = ttk.Frame(notebook)
        tab3 = ttk.Frame(notebook)
        notebook.add(tab1, text='HIL_STATE_QUATERNION')
        notebook.add(tab2, text='HIL_SENSOR')
        notebook.add(tab3, text='GPS_INPUT')

        fields_quat = ['q1', 'q2', 'q3', 'q4', 'rollspeed', 'pitchspeed', 'yawspeed', 'lat', 'lon', 'alt', 'vx', 'vy', 'vz', 'ind_airspeed', 'true_airspeed', 'xacc', 'yacc', 'zacc']
        self.message_entries['HIL_STATE_QUATERNION'] = self._create_message_frame(tab1, "HIL_STATE_QUATERNION (ID: 115)", fields_quat)
        
        fields_sensor = ['xacc', 'yacc', 'zacc', 'xgyro', 'ygyro', 'zgyro', 'xmag', 'ymag', 'zmag', 'abs_pressure', 'diff_pressure', 'pressure_alt', 'temperature', 'fields_updated']
        self.message_entries['HIL_SENSOR'] = self._create_message_frame(tab2, "HIL_SENSOR (ID: 107)", fields_sensor)

        fields_gps_input = ['gps_id', 'ignore_flags', 'time_week_ms', 'time_week', 'fix_type', 'lat', 'lon', 'alt', 'hdop', 'vdop', 'vn', 've', 'vd', 'speed_accuracy', 'horiz_accuracy', 'vert_accuracy', 'satellites_visible', 'yaw']
        self.message_entries['GPS_INPUT'] = self._create_message_frame(tab3, "GPS_INPUT (ID: 232)", fields_gps_input)

    def _create_message_frame(self, parent, title, fields):
        msg_name = title.split(' ')[0]
        frame = ttk.Frame(parent, padding="10")
        frame.pack(fill=tk.BOTH, expand=True)

        title_label = ttk.Label(frame, text=title, font=('Helvetica', 10, 'bold'))
        title_label.pack(anchor='w', pady=(0, 10))
        if msg_name in self.MAVLINK_MESSAGE_INFO:
            CreateToolTip(title_label, self.MAVLINK_MESSAGE_INFO[msg_name])
        
        main_content_frame = ttk.Frame(frame)
        main_content_frame.pack(fill=tk.BOTH, expand=True, anchor='n')

        fields_frame = ttk.Frame(main_content_frame)
        fields_frame.pack(side=tk.TOP, fill=tk.X, anchor='n')

        helper_frame_container = ttk.Frame(main_content_frame)
        helper_frame_container.pack(side=tk.TOP, fill=tk.X, anchor='n', pady=(10, 0))

        entries = {}
        num_columns = 3 
        for i, field in enumerate(fields):
            row, col = i // num_columns, i % num_columns
            
            label = ttk.Label(fields_frame, text=f"{field}:")
            label.grid(row=row, column=2*col, sticky="w", padx=2, pady=2)
            
            field_key = f"{msg_name}_{field}" if f"{msg_name}_{field}" in self.MAVLINK_FIELD_INFO else field
            field_info = self.MAVLINK_FIELD_INFO.get(field_key)
            if field_info:
                tooltip_text = f"Description: {field_info['desc']}"
                if field_info.get('unit'): tooltip_text += f"\nUnit: {field_info['unit']}"
                if field_info.get('type'): tooltip_text += f"\nType: {field_info['type']}"
                CreateToolTip(label, tooltip_text)

            if msg_name == 'GPS_INPUT' and field == 'fix_type':
                fix_type_map = {"0: No Fix": 0, "2: 2D Fix": 2, "3: 3D Fix": 3, "4: DGPS": 4, "5: RTK": 5}
                combo = ttk.Combobox(fields_frame, values=list(fix_type_map.keys()), state="readonly", width=12)
                combo.set("3: 3D Fix")
                combo.grid(row=row, column=2*col + 1, sticky="ew", padx=5, pady=2)
                entries[field], entries['_fix_type_map'] = combo, fix_type_map
            else:
                entry = ttk.Entry(fields_frame, width=12)
                entry.insert(0, "0")
                if field in ['fields_updated', 'ignore_flags']: entry.config(state='readonly')
                entry.grid(row=row, column=2*col + 1, sticky="ew", padx=5, pady=2)
                entries[field] = entry
        
        if msg_name == 'HIL_STATE_QUATERNION': self._create_quaternion_helper(helper_frame_container, entries)
        if msg_name == 'HIL_SENSOR': self._create_sensor_flags_helper(helper_frame_container, entries['fields_updated'])
        if msg_name == 'GPS_INPUT': self._create_gps_input_flags_helper(helper_frame_container, entries['ignore_flags'])
        
        control_frame = ttk.Frame(frame)
        control_frame.pack(fill=tk.X, expand=False, pady=(15,0), side=tk.BOTTOM)
        send_once_btn = ttk.Button(control_frame, text="1회 전송", state=tk.DISABLED, command=lambda name=msg_name: self._send_once(name))
        send_once_btn.pack(side=tk.LEFT, padx=5, expand=True, fill=tk.X)
        periodic_btn = ttk.Button(control_frame, text="주기 전송 시작", state=tk.DISABLED, command=lambda name=msg_name: self._toggle_periodic_send(name))
        periodic_btn.pack(side=tk.LEFT, padx=5, expand=True, fill=tk.X)
        hz_label = ttk.Label(control_frame, text="주파수(Hz):")
        hz_label.pack(side=tk.LEFT, padx=(5,0))
        hz_entry = ttk.Entry(control_frame, width=5)
        hz_entry.insert(0, "10"); hz_entry.pack(side=tk.LEFT, padx=(0,5))
        entries['_controls'] = {'send_once': send_once_btn, 'periodic': periodic_btn, 'hz_entry': hz_entry}
        return entries

    def _create_quaternion_helper(self, parent, quat_entries):
        helper_frame = ttk.LabelFrame(parent, text="Euler to Quaternion")
        helper_frame.pack(fill=tk.X, expand=True)
        
        ttk.Label(helper_frame, text="Roll(deg):").pack(side=tk.LEFT, padx=2)
        roll_entry = ttk.Entry(helper_frame, width=6); roll_entry.insert(0,"0"); roll_entry.pack(side=tk.LEFT, padx=2)
        ttk.Label(helper_frame, text="Pitch(deg):").pack(side=tk.LEFT, padx=2)
        pitch_entry = ttk.Entry(helper_frame, width=6); pitch_entry.insert(0,"0"); pitch_entry.pack(side=tk.LEFT, padx=2)
        ttk.Label(helper_frame, text="Yaw(deg):").pack(side=tk.LEFT, padx=2)
        yaw_entry = ttk.Entry(helper_frame, width=6); yaw_entry.insert(0,"0"); yaw_entry.pack(side=tk.LEFT, padx=2)
        
        def convert():
            try:
                roll, pitch, yaw = map(lambda x: math.radians(float(x.get())), [roll_entry, pitch_entry, yaw_entry])
                cy, sy = math.cos(yaw*0.5), math.sin(yaw*0.5)
                cp, sp = math.cos(pitch*0.5), math.sin(pitch*0.5)
                cr, sr = math.cos(roll*0.5), math.sin(roll*0.5)
                q = [cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy]
                for i, val in enumerate(q):
                    quat_entries[f'q{i+1}'].delete(0, tk.END); quat_entries[f'q{i+1}'].insert(0, f"{val:.8f}")
            except ValueError: self._log_tx("ERROR: Roll/Pitch/Yaw에 유효한 숫자를 입력하십시오.")
        ttk.Button(helper_frame, text="변환", command=convert).pack(side=tk.RIGHT, padx=5)

    def _create_sensor_flags_helper(self, parent, target_entry):
        helper_frame = ttk.LabelFrame(parent, text="HIL_SENSOR_UPDATED_FLAGS")
        helper_frame.pack(fill=tk.X, expand=True)
        
        flags = {"ACC": 7, "GYRO": 56, "MAG": 448, "ABS_PRESS": 512, "DIFF_PRESS": 1024, "PRESS_ALT": 2048, "TEMP": 4096, "RESET": 2147483648}
        flag_vars = {}
        def update_mask():
            mask = sum(flags[name] for name, var in flag_vars.items() if var.get())
            target_entry.config(state='normal'); target_entry.delete(0, tk.END)
            target_entry.insert(0, str(mask)); target_entry.config(state='readonly')
        
        for i, (name, val) in enumerate(flags.items()):
            var = tk.IntVar(value=0)
            chk = ttk.Checkbutton(helper_frame, text=name, variable=var, command=update_mask)
            chk.pack(side=tk.LEFT, padx=3, fill=tk.X, expand=True)
            flag_vars[name] = var
        update_mask()

    def _create_gps_input_flags_helper(self, parent, target_entry):
        helper_frame = ttk.LabelFrame(parent, text="GPS_INPUT_IGNORE_FLAGS")
        helper_frame.pack(fill=tk.X, expand=True)
        
        flags = {
            "FLAG_ALT": 1, "FLAG_HDOP": 2, "FLAG_VDOP": 4, "FLAG_VEL_HORIZ": 8,
            "FLAG_VEL_VERT": 16, "FLAG_SPEED_ACCURACY": 32, "FLAG_HORIZ_ACCURACY": 64,
            "FLAG_VERT_ACCURACY": 128, "FLAG_YAW": 256
        }
        flag_vars = {}
        def update_mask():
            mask = sum(flags[name] for name, var in flag_vars.items() if var.get())
            target_entry.config(state='normal'); target_entry.delete(0, tk.END)
            target_entry.insert(0, str(mask)); target_entry.config(state='readonly')
        
        for name, val in flags.items():
            var = tk.IntVar(value=0)
            chk = ttk.Checkbutton(helper_frame, text=name.replace("FLAG_", ""), variable=var, command=update_mask)
            chk.pack(side=tk.LEFT, padx=2, fill=tk.X, expand=True)
            flag_vars[name] = var
        update_mask()

    def _create_log_widgets(self, parent):
        tx_log_frame = ttk.LabelFrame(parent, text="송신(TX) & 시스템 로그", padding="5")
        tx_log_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, pady=(0, 5))
        self.tx_log_text = scrolledtext.ScrolledText(tx_log_frame, state='disabled', height=8, wrap=tk.WORD)
        self.tx_log_text.pack(fill=tk.BOTH, expand=True)
        
        rx_log_frame = ttk.LabelFrame(parent, text="수신(RX) 로그", padding="5")
        rx_log_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        self.rx_log_text = scrolledtext.ScrolledText(rx_log_frame, state='disabled', height=8, wrap=tk.WORD)
        self.rx_log_text.pack(fill=tk.BOTH, expand=True)

    def _get_com_ports(self): return [port.device for port in serial.tools.list_ports.comports()]
    def _toggle_connection(self): self._disconnect() if self.mav_connection else self._connect()

    def _connect(self):
        port, baud = self.com_port_combo.get(), self.baud_rate_combo.get()
        if not port: return self._log_tx("ERROR: COM 포트를 선택하십시오.")
        try:
            self.mav_connection = mavutil.mavlink_connection(port, baud=int(baud))
            self.mav_connection.wait_heartbeat()
            self._log_tx(f"SYSTEM: {port} @ {baud} 연결 성공. (SysID: {self.mav_connection.target_system})")
            self.is_running.set()
            threading.Thread(target=self._receiver_worker, daemon=True).start()
            self.connect_btn.config(text="연결 해제"); self._update_button_states(tk.NORMAL)
        except Exception as e:
            self.mav_connection = None; self._log_tx(f"ERROR: 연결 실패. ({e})")

    def _disconnect(self):
        for msg_name in self.periodic_senders:
            if 'event' in self.periodic_senders[msg_name] and self.periodic_senders[msg_name]['event'].is_set(): self.periodic_senders[msg_name]['event'].clear()
        self.is_running.clear()
        if self.mav_connection: self.mav_connection.close(); self.mav_connection = None
        self._log_tx("SYSTEM: 연결이 해제되었습니다."); self.connect_btn.config(text="연결"); self._update_button_states(tk.DISABLED)

    def _update_button_states(self, state):
        for msg in self.message_entries.values():
            controls = msg['_controls']
            controls['send_once'].config(state=state); controls['periodic'].config(state=state)
            if state == tk.DISABLED: controls['periodic'].config(text="주기 전송 시작")

    def _get_values_from_entries(self, entries):
        vals = {}
        for key, widget in entries.items():
            if key.startswith('_'): continue
            val_str = widget.get()
            try:
                if key == 'fix_type': vals[key] = entries['_fix_type_map'][val_str]
                elif key in ['lat', 'lon'] and float(val_str) != 0: 
                    vals[key] = int(float(val_str) * 1e7)
                elif key == 'alt' and 'GPS_INPUT' in str(type(entries)):
                    vals[key] = float(val_str)
                elif key in ['lat', 'lon', 'alt'] and float(val_str) != 0:
                    if key == 'alt': vals[key] = int(float(val_str) * 1000)
                    else: vals[key] = int(float(val_str) * 1e7)
                else: vals[key] = float(val_str)
            except (ValueError, KeyError): self._log_tx(f"ERROR: '{key}' 필드에 유효한 값을 입력/선택하십시오."); return None
        return vals
        
    def _send_once(self, msg_name):
        if not self.mav_connection: return
        vals = self._get_values_from_entries(self.message_entries[msg_name])
        if not vals: return
        
        time_usec = int(time.time_ns() / 1000)
        try:
            if msg_name == 'HIL_SENSOR':
                self.mav_connection.mav.hil_sensor_send(time_usec, vals['xacc'], vals['yacc'], vals['zacc'], vals['xgyro'], vals['ygyro'], vals['zgyro'], vals['xmag'], vals['ymag'], vals['zmag'], vals['abs_pressure'], vals['diff_pressure'], vals['pressure_alt'], vals['temperature'], int(vals['fields_updated']))
            elif msg_name == 'GPS_INPUT':
                self.mav_connection.mav.gps_input_send(
                    time_usec, 
                    int(vals.get('gps_id', 0)), 
                    int(vals.get('ignore_flags', 0)), 
                    int(vals.get('time_week_ms', 0)), 
                    int(vals.get('time_week', 0)), 
                    int(vals.get('fix_type', 3)), 
                    int(vals.get('lat', 0)), 
                    int(vals.get('lon', 0)), 
                    float(vals.get('alt', 0.0)), 
                    float(vals.get('hdop', 1.0)), 
                    float(vals.get('vdop', 1.0)), 
                    float(vals.get('vn', 0.0)), 
                    float(vals.get('ve', 0.0)), 
                    float(vals.get('vd', 0.0)), 
                    float(vals.get('speed_accuracy', 0.0)), 
                    float(vals.get('horiz_accuracy', 0.0)), 
                    float(vals.get('vert_accuracy', 0.0)), 
                    int(vals.get('satellites_visible', 10)),
                    int(vals.get('yaw', 0))
                )
            elif msg_name == 'HIL_STATE_QUATERNION':
                self.mav_connection.mav.hil_state_quaternion_send(time_usec, [vals['q1'],vals['q2'],vals['q3'],vals['q4']], vals['rollspeed'], vals['pitchspeed'], vals['yawspeed'], int(vals['lat']), int(vals['lon']), int(vals['alt']), int(vals.get('vx',0)*100), int(vals.get('vy',0)*100), int(vals.get('vz',0)*100), int(vals.get('ind_airspeed',0)*100), int(vals.get('true_airspeed',0)*100), int(vals.get('xacc',0)*1000), int(vals.get('yacc',0)*1000), int(vals.get('zacc',0)*1000))
            self._log_tx(f"TX: {msg_name} 메시지를 전송했습니다.")
        except Exception as e: self._log_tx(f"ERROR: {msg_name} 전송 중 오류 발생: {e}")

    def _toggle_periodic_send(self, msg_name):
        sender = self.periodic_senders.setdefault(msg_name, {'event': threading.Event()})
        event, button = sender['event'], self.message_entries[msg_name]['_controls']['periodic']
        if event.is_set():
            event.clear(); button.config(text="주기 전송 시작"); self._log_tx(f"SYSTEM: {msg_name} 주기 전송을 중지했습니다.")
        else:
            try:
                hz = float(self.message_entries[msg_name]['_controls']['hz_entry'].get())
                if hz <= 0: raise ValueError("Hz는 0보다 커야 합니다.")
                event.set()
                sender['thread'] = threading.Thread(target=self._periodic_sender_worker, args=(msg_name, 1.0/hz), daemon=True).start()
                button.config(text="주기 전송 중지"); self._log_tx(f"SYSTEM: {msg_name} 메시지를 {hz}Hz 주기로 전송 시작.")
            except ValueError as e: self._log_tx(f"ERROR: Hz 값 오류: {e}")
    
    def _periodic_sender_worker(self, msg_name, period):
        event = self.periodic_senders[msg_name]['event']
        while event.is_set(): self._send_once(msg_name); time.sleep(period)

    def _receiver_worker(self):
        while self.is_running.is_set():
            try:
                msg = self.mav_connection.recv_match(type=['SERVO_OUTPUT_RAW', 'HEARTBEAT'], blocking=True, timeout=1)
                if msg:
                    self.gui_queue.put(('ui_update', msg))
                    self.gui_queue.put(('log_rx', f"{msg.get_type()} | {msg.to_dict()}"))
            except Exception as e:
                if self.is_running.is_set(): self.gui_queue.put(('log_tx', f"ERROR: 수신 중 오류: {e}")); break

    def _process_gui_queue(self):
        while not self.gui_queue.empty():
            try:
                task, data = self.gui_queue.get_nowait()
                if task == 'log_tx': self._write_to_log(self.tx_log_text, data)
                elif task == 'log_rx': self._write_to_log(self.rx_log_text, data, trim=True)
                elif task == 'ui_update': self._update_realtime_status(data)
            except queue.Empty: pass
        self.master.after(100, self._process_gui_queue)

    def _update_realtime_status(self, msg):
        msg_type = msg.get_type()
        if msg_type == 'HEARTBEAT':
            self.hb_vars['Mode'].set(mavutil.mode_string_v10(msg))
            self.hb_vars['System Status'].set(mavutil.mavlink.enums['MAV_STATE'][msg.system_status].name.replace('MAV_STATE_', ''))
            self.hb_vars['Type'].set(mavutil.mavlink.enums['MAV_TYPE'][msg.type].name.replace('MAV_TYPE_', ''))
            self.hb_vars['Autopilot'].set(mavutil.mavlink.enums['MAV_AUTOPILOT'][msg.autopilot].name.replace('MAV_AUTOPILOT_', ''))
        elif msg_type == 'SERVO_OUTPUT_RAW':
            servos = [msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw, msg.servo5_raw, msg.servo6_raw, msg.servo7_raw, msg.servo8_raw]
            for i in range(8): self.servo_vars[i].set(str(servos[i]))

    def _log_tx(self, message):
        self.gui_queue.put(('log_tx', message))

    def _log_rx(self, message):
        self.gui_queue.put(('log_rx', message))

    def _write_to_log(self, log_widget, message, trim=False):
        log_widget.config(state=tk.NORMAL)
        timestamp = datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]
        log_widget.insert(tk.END, f"{timestamp} - {message}\n")
        log_widget.see(tk.END)
        if trim and int(log_widget.index('end-1c').split('.')[0]) > self.MAX_LOG_LINES:
            log_widget.delete('1.0', f'{int(log_widget.index("end-1c").split(".")[0]) - self.MAX_LOG_LINES}.0')
        log_widget.config(state=tk.DISABLED)

    def _on_closing(self):
        if self.mav_connection: self._disconnect()
        self.master.destroy()

if __name__ == '__main__':
    root = tk.Tk()
    app = AdvancedMAVLinkHILGUI(root)
    root.mainloop()