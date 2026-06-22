#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk
import importlib
import math
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy

class JoyGuiPublisher(Node):
    def __init__(self) -> None:
        super().__init__('joy_gui_publisher')

        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('odom_topic', '/drone/odometry')
        self.declare_parameter('position_target_topic', '/drone/position_target')
        self.declare_parameter('control_mode_topic', '/drone/control_mode')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('control_mode_service', '/drone/set_control_mode')
        self.declare_parameter('position_target_service', '/drone/set_position_target')

        self._joy_topic = self.get_parameter('joy_topic').value or '/joy'
        self._odom_topic = self.get_parameter('odom_topic').value or '/drone/odometry'
        self._position_target_topic = self.get_parameter('position_target_topic').value or '/drone/position_target'
        self._control_mode_topic = self.get_parameter('control_mode_topic').value or '/drone/control_mode'
        self._publish_rate_hz = float(self.get_parameter('publish_rate_hz').value or 20.0)
        self._publish_period_ms = max(20, int(1000.0 / self._publish_rate_hz))
        self._control_mode_service = self.get_parameter('control_mode_service').value or '/drone/set_control_mode'
        self._position_target_service = self.get_parameter('position_target_service').value or '/drone/set_position_target'

        self._publisher = self.create_publisher(Joy, self._joy_topic, 10)
        self._control_mode_client = None
        self._position_target_client = None
        self._odom_subscription = self.create_subscription(
            Odometry,
            self._odom_topic,
            self._odom_callback,
            10,
        )
        self._target_subscription = self.create_subscription(
            Odometry,
            self._position_target_topic,
            self._target_callback,
            10,
        )
        self._mode_subscription = self.create_subscription(
            String,
            self._control_mode_topic,
            self._control_mode_callback,
            10,
        )

        self._window = tk.Tk()
        self._window.title('Drone Control')
        self._window.geometry('780x560')
        self._window.minsize(720, 520)
        self._window.protocol('WM_DELETE_WINDOW', self._on_close)
        self._window.configure(bg='#0f172a')

        self._roll = tk.DoubleVar(value=0.0)
        self._pitch = tk.DoubleVar(value=0.0)
        self._yaw = tk.DoubleVar(value=0.0)
        self._throttle = tk.DoubleVar(value=-1.0)
        self._deadman = tk.IntVar(value=1)
        self._deadman_state = tk.StringVar(value='ON')
        self._mode_value = 0
        self._mode_label = tk.StringVar(value='manual_attitude')
        self._status = tk.StringVar(value='Ready')
        self._service_state = tk.StringVar(value='Services: connecting...')
        self._pose_state = tk.StringVar(value='Position: waiting for odometry...')
        self._orientation_state = tk.StringVar(value='Orientation: waiting for odometry...')
        self._target_state = tk.StringVar(value='Target: waiting for hold target...')
        self._target_note = tk.StringVar(value='Target editing is enabled in position hold mode.')
        self._target_x_text = tk.StringVar(value='0.00')
        self._target_y_text = tk.StringVar(value='0.00')
        self._target_z_text = tk.StringVar(value='0.00')
        self._target_yaw_text = tk.StringVar(value='0.0')
        self._have_odom = False
        self._current_x = 0.0
        self._current_y = 0.0
        self._current_z = 0.0
        self._current_roll = 0.0
        self._current_pitch = 0.0
        self._current_yaw = 0.0
        self._current_yaw_deg = 0.0
        self._target_x = 0.0
        self._target_y = 0.0
        self._target_z = 0.0
        self._target_yaw = 0.0

        self._setup_styles()
        self._build_ui()
        self._set_mode_ui('manual_attitude')
        self._window.bind('<space>', lambda _event: self._toggle_deadman())
        self._window.bind('m', lambda _event: self._toggle_mode())
        self._window.bind('r', lambda _event: self._reset_target_to_current())
        self._window.bind('0', lambda _event: self._reset())

    def _setup_styles(self) -> None:
        style = ttk.Style(self._window)
        style.theme_use('clam')
        style.configure('Root.TFrame', background='#0f172a')
        style.configure('Card.TFrame', background='#111827', relief='flat')
        style.configure('Title.TLabel', background='#0f172a', foreground='#e5e7eb', font=('TkDefaultFont', 18, 'bold'))
        style.configure('Subtitle.TLabel', background='#0f172a', foreground='#94a3b8', font=('TkDefaultFont', 10))
        style.configure('CardTitle.TLabel', background='#111827', foreground='#f8fafc', font=('TkDefaultFont', 11, 'bold'))
        style.configure('CardText.TLabel', background='#111827', foreground='#cbd5e1', font=('TkDefaultFont', 10))
        style.configure('Mode.TLabel', background='#111827', foreground='#38bdf8', font=('TkDefaultFont', 11, 'bold'))
        style.configure('Value.TLabel', background='#111827', foreground='#e2e8f0', font=('TkDefaultFont', 9, 'bold'))
        style.configure('Accent.TButton', padding=(10, 7), font=('TkDefaultFont', 10, 'bold'))
        style.configure('Small.TButton', padding=(8, 5))
        style.map('Accent.TButton', foreground=[('active', '#ffffff')])

    def _build_ui(self) -> None:
        container = ttk.Frame(self._window, style='Root.TFrame', padding=14)
        container.pack(fill='both', expand=True)

        ttk.Label(container, text='Drone Control', style='Title.TLabel').pack(anchor='w')
        ttk.Label(
            container,
            text='Space toggles deadman, M toggles mode, R loads the current pose into the hold target, 0 resets axes.',
            style='Subtitle.TLabel',
        ).pack(anchor='w', pady=(4, 12))

        top_row = ttk.Frame(container, style='Root.TFrame')
        top_row.pack(fill='x', pady=(0, 10))
        self._build_mode_card(top_row)
        self._build_telemetry_card(top_row)

        bottom_row = ttk.Frame(container, style='Root.TFrame')
        bottom_row.pack(fill='both', expand=True)
        self._build_target_card(bottom_row)
        self._build_axes_card(bottom_row)

        footer = ttk.Frame(container, style='Root.TFrame')
        footer.pack(fill='x', pady=(10, 0))
        ttk.Label(footer, textvariable=self._status, style='Subtitle.TLabel').pack(anchor='w')

    def _build_mode_card(self, parent: ttk.Frame) -> None:
        card = ttk.Frame(parent, style='Card.TFrame', padding=12)
        card.pack(side='left', fill='both', expand=True, padx=(0, 6))

        ttk.Label(card, text='Mode', style='CardTitle.TLabel').pack(anchor='w')
        ttk.Label(card, textvariable=self._mode_label, style='Mode.TLabel').pack(anchor='w', pady=(2, 8))

        row = ttk.Frame(card, style='Card.TFrame')
        row.pack(fill='x')
        self._manual_mode_button = ttk.Button(
            row,
            text='Manual',
            style='Accent.TButton',
            command=lambda: self._set_control_mode(0, 'manual_attitude'),
        )
        self._manual_mode_button.pack(side='left', padx=(0, 6))
        self._position_mode_button = ttk.Button(
            row,
            text='Position Hold',
            style='Accent.TButton',
            command=lambda: self._set_control_mode(1, 'position_hold'),
        )
        self._position_mode_button.pack(side='left')

        ttk.Label(card, textvariable=self._service_state, style='CardText.TLabel', wraplength=260, justify='left').pack(anchor='w', pady=(8, 0))

    def _build_telemetry_card(self, parent: ttk.Frame) -> None:
        card = ttk.Frame(parent, style='Card.TFrame', padding=12)
        card.pack(side='left', fill='both', expand=True, padx=(6, 0))

        ttk.Label(card, text='Telemetry', style='CardTitle.TLabel').pack(anchor='w')
        box = ttk.Frame(card, style='Card.TFrame')
        box.pack(fill='x', pady=(6, 0))
        ttk.Label(box, textvariable=self._pose_state, style='CardText.TLabel', wraplength=260, justify='left').pack(anchor='w', pady=(0, 4))
        ttk.Label(box, textvariable=self._orientation_state, style='CardText.TLabel', wraplength=260, justify='left').pack(anchor='w', pady=(0, 4))
        ttk.Label(box, textvariable=self._target_state, style='CardText.TLabel', wraplength=260, justify='left').pack(anchor='w')

    def _build_target_card(self, parent: ttk.Frame) -> None:
        card = ttk.Frame(parent, style='Card.TFrame', padding=12)
        card.pack(side='left', fill='both', expand=True, padx=(0, 6))

        ttk.Label(card, text='Position Hold Target', style='CardTitle.TLabel').pack(anchor='w')
        ttk.Label(
            card,
            text='Edit the target and press Apply. The controller will use the current pose target while in position hold.',
            style='CardText.TLabel',
            wraplength=260,
            justify='left',
        ).pack(anchor='w', pady=(2, 8))
        ttk.Label(card, textvariable=self._target_note, style='CardText.TLabel', wraplength=260, justify='left').pack(anchor='w', pady=(0, 8))

        grid = ttk.Frame(card, style='Card.TFrame')
        grid.pack(fill='x')
        self._target_entry_widgets: list[ttk.Entry] = [
            self._add_target_entry(grid, 0, 'X', self._target_x_text),
            self._add_target_entry(grid, 1, 'Y', self._target_y_text),
            self._add_target_entry(grid, 2, 'Z', self._target_z_text),
            self._add_target_entry(grid, 3, 'Yaw', self._target_yaw_text),
        ]

        row = ttk.Frame(card, style='Card.TFrame')
        row.pack(fill='x', pady=(8, 0))
        self._apply_target_button = ttk.Button(row, text='Apply Target', style='Accent.TButton', command=self._apply_target_from_fields)
        self._apply_target_button.pack(side='left', padx=(0, 6))
        self._load_current_button = ttk.Button(row, text='Load Current', style='Small.TButton', command=self._reset_target_to_current)
        self._load_current_button.pack(side='left')

    def _build_axes_card(self, parent: ttk.Frame) -> None:
        card = ttk.Frame(parent, style='Card.TFrame', padding=12)
        card.pack(fill='both', expand=True)

        ttk.Label(card, text='Axes', style='CardTitle.TLabel').pack(anchor='w')
        self._add_axis_slider(card, 'Roll', self._roll)
        self._add_axis_slider(card, 'Pitch', self._pitch)
        self._add_axis_slider(card, 'Yaw', self._yaw)
        self._add_axis_slider(card, 'Throttle', self._throttle)

        row = ttk.Frame(card, style='Card.TFrame')
        row.pack(fill='x', pady=(8, 0))
        ttk.Checkbutton(row, text='Deadman', variable=self._deadman).pack(side='left')
        ttk.Label(row, text='0 resets axes', style='CardText.TLabel').pack(side='right')

    def _add_axis_slider(self, parent: ttk.Frame, label: str, variable: tk.DoubleVar) -> None:
        frame = ttk.Frame(parent, style='Card.TFrame')
        frame.pack(fill='x', pady=4)
        ttk.Label(frame, text=label, style='CardText.TLabel', width=10).pack(side='left')
        scale = ttk.Scale(frame, from_=-1.0, to=1.0, variable=variable)
        scale.pack(side='left', fill='x', expand=True, padx=(6, 6))
        value = ttk.Label(frame, text=f'{variable.get():.2f}', style='Value.TLabel', width=6)
        value.pack(side='right')

        def sync_value(*_args: object) -> None:
            value.configure(text=f'{variable.get():.2f}')

        variable.trace_add('write', sync_value)
        sync_value()

    def _add_target_entry(self, parent: ttk.Frame, row: int, label: str, variable: tk.StringVar) -> ttk.Entry:
        ttk.Label(parent, text=label, style='CardText.TLabel').grid(row=row, column=0, sticky='w', pady=2)
        entry = ttk.Entry(parent, textvariable=variable, width=10)
        entry.grid(row=row, column=1, sticky='w', padx=(6, 0), pady=2)
        return entry

    def _read_float_var(self, variable: tk.StringVar, label: str) -> Optional[float]:
        try:
            return float(variable.get())
        except ValueError:
            self._set_feedback(f'Invalid {label} value', f'Invalid target input for {label}.')
            return None

    def _build_message(self) -> Joy:
        message = Joy()
        message.axes = [
            float(self._roll.get()),
            float(self._pitch.get()),
            float(self._yaw.get()),
            float(self._throttle.get()),
        ]
        buttons = [0, 0, 0, 0, int(self._deadman.get()), 0, 0, 0]
        message.buttons = buttons
        return message

    def _publish(self) -> None:
        message = self._build_message()
        self._publisher.publish(message)
        self._deadman_state.set('ON' if self._deadman.get() else 'OFF')
        self._status.set('Publishing roll={:.2f} pitch={:.2f} yaw={:.2f} throttle={:.2f}'.format(
            message.axes[0], message.axes[1], message.axes[2], message.axes[3]
        ))

    def _reset(self) -> None:
        self._roll.set(0.0)
        self._pitch.set(0.0)
        self._yaw.set(0.0)
        self._throttle.set(-1.0)
        self._deadman.set(0)
        self._publish()

    def _toggle_deadman(self) -> None:
        self._deadman.set(0 if self._deadman.get() else 1)
        self._publish()

    def _toggle_mode(self) -> None:
        target_mode = 1 if self._mode_value == 0 else 0
        target_name = 'position_hold' if target_mode == 1 else 'manual_attitude'
        self._set_control_mode(target_mode, target_name)

    def _set_feedback(self, service_text: str, status_text: Optional[str] = None, mode_text: Optional[str] = None) -> None:
        self._service_state.set(service_text)
        if status_text is not None:
            self._status.set(status_text)
        if mode_text is not None:
            self._mode_label.set(mode_text)

    def _set_mode_ui(self, mode_name: str) -> None:
        normalized_mode = (mode_name or 'manual_attitude').strip().lower()
        is_position_hold = normalized_mode == 'position_hold'
        self._mode_value = 1 if is_position_hold else 0
        self._mode_label.set(normalized_mode)

        self._manual_mode_button.state(['disabled'] if self._mode_value == 0 else ['!disabled'])
        self._position_mode_button.state(['disabled'] if self._mode_value == 1 else ['!disabled'])

        target_state = ['!disabled'] if is_position_hold else ['disabled']
        self._apply_target_button.state(target_state)
        self._load_current_button.state(target_state)
        self._target_note.set(
            'Target editing is enabled in position hold mode.' if is_position_hold
            else 'Switch to position hold to apply or load a target.'
        )
        for widget in self._target_entry_widgets:
            widget.state(target_state)

    def _set_control_mode(self, mode: int, mode_name: str) -> None:
        service_type = self._load_service_type('SetControlMode')
        if service_type is None:
            self._set_feedback('control mode service type unavailable', 'Control mode service type is unavailable.')
            return

        client = self._ensure_control_mode_client(service_type)
        if client is None or not client.wait_for_service(timeout_sec=0.1):
            self._set_feedback('control mode service unavailable', 'Control mode service is not available.')
            return

        request = service_type.Request()
        request.mode = mode
        future = client.call_async(request)
        self._set_feedback(f'sending mode: {mode_name}')

        def on_done(done_future) -> None:
            try:
                response = done_future.result()
            except Exception as exc:  # pragma: no cover - runtime safety
                self._window.after(0, lambda: self._set_feedback(str(exc), 'Control mode request failed.'))
                return

            def apply_response() -> None:
                self._mode_value = mode
                self._set_feedback(
                    response.message or f'Control mode set to {mode_name}',
                    f'Control mode: {mode_name}',
                    mode_name,
                )
                self._set_mode_ui(mode_name)

            self._window.after(0, apply_response)

        future.add_done_callback(on_done)

    def _reset_target_to_current(self) -> None:
        if not self._have_odom:
            self._set_feedback('odometry unavailable', 'Cannot reset target before odometry is available.')
            return

        self._target_x_text.set(f'{self._current_x:.2f}')
        self._target_y_text.set(f'{self._current_y:.2f}')
        self._target_z_text.set(f'{self._current_z:.2f}')
        self._target_yaw_text.set(f'{self._current_yaw_deg:.1f}')
        self._set_position_target(self._current_x, self._current_y, self._current_z, self._current_yaw_deg)

    def _apply_target_from_fields(self) -> None:
        x = self._read_float_var(self._target_x_text, 'X')
        y = self._read_float_var(self._target_y_text, 'Y')
        z = self._read_float_var(self._target_z_text, 'Z')
        yaw_deg = self._read_float_var(self._target_yaw_text, 'Yaw')
        if x is None or y is None or z is None or yaw_deg is None:
            return

        self._set_position_target(x, y, z, yaw_deg)

    def _set_position_target(self, x: float, y: float, z: float, yaw_deg: float) -> None:
        service_type = self._load_service_type('SetPositionTarget')
        if service_type is None:
            self._set_feedback('position target service type unavailable', 'Position target service type is unavailable.')
            return

        client = self._ensure_position_target_client(service_type)
        if client is None or not client.wait_for_service(timeout_sec=0.1):
            self._set_feedback('position target service unavailable', 'Position target service is not available.')
            return

        request = service_type.Request()
        request.x = x
        request.y = y
        request.z = z
        request.yaw_deg = yaw_deg
        future = client.call_async(request)
        self._set_feedback('sending target...')

        def on_done(done_future) -> None:
            try:
                response = done_future.result()
            except Exception as exc:  # pragma: no cover - runtime safety
                self._window.after(0, lambda: self._set_feedback(str(exc), 'Position target request failed.'))
                return

            self._window.after(0, lambda: self._set_feedback(
                response.message or 'Position target set',
                f'Position target updated: x={x:.2f} y={y:.2f} z={z:.2f} yaw={yaw_deg:.1f}°',
                self._mode_label.get(),
            ))
            self._window.after(0, lambda: self._target_state.set(
                f'Target: x={x:.2f} y={y:.2f} z={z:.2f} yaw={yaw_deg:.1f}°'
            ))

        future.add_done_callback(on_done)

    def _odom_callback(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        roll, pitch, yaw = self._quaternion_to_euler(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )

        self._have_odom = True
        self._current_x = position.x
        self._current_y = position.y
        self._current_z = position.z
        self._current_roll = roll
        self._current_pitch = pitch
        self._current_yaw = yaw
        self._current_yaw_deg = math.degrees(yaw)
        self._pose_state.set(f'Position: x={position.x:.2f} y={position.y:.2f} z={position.z:.2f}')
        self._orientation_state.set(
            'Orientation: roll={:.1f}° pitch={:.1f}° yaw={:.1f}°'.format(
                math.degrees(roll),
                math.degrees(pitch),
                math.degrees(yaw),
            )
        )

    def _target_callback(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        _, _, yaw = self._quaternion_to_euler(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )

        self._target_x = position.x
        self._target_y = position.y
        self._target_z = position.z
        self._target_yaw = yaw
        self._target_state.set(
            'Target: x={:.2f} y={:.2f} z={:.2f} yaw={:.1f}°'.format(
                position.x,
                position.y,
                position.z,
                math.degrees(yaw),
            )
        )

    def _control_mode_callback(self, msg: String) -> None:
        mode_name = (msg.data or 'manual_attitude').strip().lower()
        self._window.after(0, lambda: self._set_mode_ui(mode_name))

    @staticmethod
    def _quaternion_to_euler(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    def _load_service_type(self, symbol: str):
        try:
            module = importlib.import_module('drone_control.srv')
            return getattr(module, symbol)
        except (ImportError, AttributeError):
            return None

    def _ensure_control_mode_client(self, service_type):
        if self._control_mode_client is None:
            self._control_mode_client = self.create_client(service_type, self._control_mode_service)
        return self._control_mode_client

    def _ensure_position_target_client(self, service_type):
        if self._position_target_client is None:
            self._position_target_client = self.create_client(service_type, self._position_target_service)
        return self._position_target_client

    def _on_close(self) -> None:
        self._deadman.set(0)
        self._roll.set(0.0)
        self._pitch.set(0.0)
        self._yaw.set(0.0)
        self._throttle.set(-1.0)
        self._publish()
        self._window.destroy()
        self.destroy_node()
        rclpy.shutdown()

    def spin(self) -> None:
        def tick() -> None:
            if not rclpy.ok():
                return
            rclpy.spin_once(self, timeout_sec=0.0)
            self._publish()
            self._window.after(self._publish_period_ms, tick)

        tick()
        self._window.mainloop()


def main() -> None:
    rclpy.init()
    node = JoyGuiPublisher()
    try:
        node.spin()
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()