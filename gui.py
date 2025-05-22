import tkinter as tk
from tkinter import ttk, messagebox
import logging
import queue
from controller import Controller, AGVState
from config import *

logger = logging.getLogger("AGV_GUI")

class AGVApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("AGV with Magnetic Guide Sensor")
        self.geometry("800x500")
        self.controller = Controller()
        self.command_queue = self.controller.command_queue
        self.sensor_queue = self.controller.sensor_queue
        self.is_running = False

        # GUI setup
        self.setup_gui()
        self.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.bind_keys()
        self.start_controller()
        self.after(100, self.update_gui)

    def setup_gui(self):
        main_pane = ttk.PanedWindow(self, orient=tk.HORIZONTAL)
        main_pane.pack(fill="both", expand=True, padx=5, pady=5)

        # Left pane: Controls
        left_frame = ttk.Frame(main_pane)
        main_pane.add(left_frame, weight=1)

        control_frame = ttk.LabelFrame(left_frame, text="Controls")
        control_frame.pack(fill="x", padx=5, pady=5)
        btn_style = {"width": 8, "height": 1, "bg": "#3498DB", "fg": "white", "font": ("Arial", 10)}
        tk.Button(control_frame, text="Forward", command=self.on_forward, **btn_style).grid(row=0, column=1, pady=2)
        tk.Button(control_frame, text="Left", command=self.on_left, **btn_style).grid(row=1, column=0, padx=2)
        tk.Button(control_frame, text="Stop", command=self.on_stop, bg="#E74C3C", fg="white",
                  font=("Arial", 10), width=8, height=1).grid(row=1, column=1, pady=2)
        tk.Button(control_frame, text="Right", command=self.on_right, **btn_style).grid(row=1, column=2, padx=2)
        tk.Button(control_frame, text="Backward", command=self.on_backward, **btn_style).grid(row=2, column=1, pady=2)

        self.speed_var = tk.IntVar(value=int(MAX_SPEED * 10))
        tk.Scale(left_frame, from_=0, to=330, orient=tk.HORIZONTAL, label="Speed (RPM)",
                 variable=self.speed_var, length=200).pack(pady=5)

        feedback_frame = ttk.LabelFrame(left_frame, text="Motor Feedback")
        feedback_frame.pack(fill="x", padx=5, pady=5)
        self.lbl_motor1 = tk.Label(feedback_frame, text="Motor 1: Unknown", font=("Arial", 8))
        self.lbl_motor1.pack(anchor="w", pady=2)
        self.lbl_motor2 = tk.Label(feedback_frame, text="Motor 2: Unknown", font=("Arial", 8))
        self.lbl_motor2.pack(anchor="w", pady=2)

        self.status_var = tk.StringVar(value="Connecting...")
        tk.Label(left_frame, textvariable=self.status_var, fg="red", font=("Arial", 8)).pack(pady=5)

        self.log_text = tk.Text(left_frame, height=6, width=30, font=("Arial", 8))
        self.log_text.pack(fill="x", padx=5, pady=5)
        self.log_handler = TextHandler(self.log_text)
        logger.addHandler(self.log_handler)

        # Right pane: Tasks and Sensor Data
        right_frame = ttk.Frame(main_pane)
        main_pane.add(right_frame, weight=1)

        task_frame = ttk.LabelFrame(right_frame, text="Tasks")
        task_frame.pack(fill="x", padx=5, pady=5)
        tk.Label(task_frame, text="Task 1: Line Following", font=("Arial", 10, "bold")).pack(pady=2)
        self.task1_status = tk.StringVar(value="Task 1: Stopped")
        tk.Label(task_frame, textvariable=self.task1_status, font=("Arial", 8)).pack(pady=2)
        self.task1_error = tk.StringVar(value="PID Error: 0.00")
        tk.Label(task_frame, textvariable=self.task1_error, font=("Arial", 8)).pack(pady=2)

        pid_frame = ttk.Frame(task_frame)
        pid_frame.pack(fill="x", padx=5, pady=2)
        ttk.Label(pid_frame, text="Kp:", font=("Arial", 8)).grid(row=0, column=0, padx=2, pady=2, sticky="e")
        self.kp_entry = ttk.Entry(pid_frame, width=8, font=("Arial", 8))
        self.kp_entry.grid(row=0, column=1, padx=2, pady=2)
        self.kp_entry.insert(0, str(INITIAL_PID_KP))
        ttk.Label(pid_frame, text="Ki:", font=("Arial", 8)).grid(row=0, column=2, padx=2, pady=2, sticky="e")
        self.ki_entry = ttk.Entry(pid_frame, width=8, font=("Arial", 8))
        self.ki_entry.grid(row=0, column=3, padx=2, pady=2)
        self.ki_entry.insert(0, str(INITIAL_PID_KI))
        ttk.Label(pid_frame, text="Kd:", font=("Arial", 8)).grid(row=0, column=4, padx=2, pady=2, sticky="e")
        self.kd_entry = ttk.Entry(pid_frame, width=8, font=("Arial", 8))
        self.kd_entry.grid(row=0, column=5, padx=2, pady=2)
        self.kd_entry.insert(0, str(INITIAL_PID_KD))
        tk.Button(pid_frame, text="Update PID", command=self.update_pid, bg="#F1C40F", fg="black",
                  font=("Arial", 8), width=10).grid(row=0, column=6, padx=5)

        task_controls = ttk.Frame(task_frame)
        task_controls.pack(fill="x", padx=5, pady=5)
        tk.Button(task_controls, text="Start Task 1", command=self.start_task1, bg="#1ABC9C", fg="white",
                  font=("Arial", 10), width=12, height=1).pack(side=tk.LEFT, padx=5)
        tk.Button(task_controls, text="Stop Task 1", command=self.on_stop, bg="#E74C3C", fg="white",
                  font=("Arial", 10), width=12, height=1).pack(side=tk.LEFT, padx=5)

        sensor_frame = ttk.LabelFrame(right_frame, text="Magnetic Sensor Data")
        sensor_frame.pack(fill="both", expand=True, padx=5, pady=5)

        data_display = ttk.Frame(sensor_frame)
        data_display.pack(fill="x", padx=5, pady=5)
        ttk.Label(data_display, text="Intermediate:", font=("Arial", 8)).grid(row=0, column=0, padx=2, pady=2, sticky="w")
        self.intermediate_value = ttk.Label(data_display, text="0", font=("Arial", 8, "bold"))
        self.intermediate_value.grid(row=0, column=1, padx=2, pady=2, sticky="w")
        ttk.Label(data_display, text="Location:", font=("Arial", 8)).grid(row=1, column=0, padx=2, pady=2, sticky="w")
        self.location_value = ttk.Label(data_display, text="0", font=("Arial", 8, "bold"))
        self.location_value.grid(row=1, column=1, padx=2, pady=2, sticky="w")
        ttk.Label(data_display, text="Position (Hex):", font=("Arial", 8)).grid(row=2, column=0, padx=2, pady=2, sticky="w")
        self.position_value_label = ttk.Label(data_display, text="0x0000", font=("Arial", 8, "bold"))
        self.position_value_label.grid(row=2, column=1, padx=2, pady=2, sticky="w")

        ttk.Label(sensor_frame, text="Raw Data (Hex):", font=("Arial", 8)).pack(anchor="w", padx=5, pady=2)
        self.hex_data_display = tk.Text(sensor_frame, height=3, width=40, font=("Arial", 8))
        self.hex_data_display.pack(fill="x", padx=5, pady=2)

        ttk.Label(sensor_frame, text="Sensors:", font=("Arial", 8)).pack(anchor="w", padx=5, pady=2)
        self.sensor_canvas = tk.Canvas(sensor_frame, width=400, height=60, bg="white")
        self.sensor_canvas.pack(fill="x", padx=5, pady=2)
        self.sensor_points = []
        for i in range(16):
            x = 20 + i * 24
            y = 30
            circle = self.sensor_canvas.create_oval(x-8, y-8, x+8, y+8, fill="white", outline="black")
            text = self.sensor_canvas.create_text(x, y, text=str(i+1), font=("Arial", 8))
            self.sensor_points.append((circle, text))

    def bind_keys(self):
        self.bind("<Up>", lambda e: self.on_forward())
        self.bind("<Down>", lambda e: self.on_backward())
        self.bind("<Left>", lambda e: self.on_left())
        self.bind("<Right>", lambda e: self.on_right())
        self.bind("<space>", lambda e: self.on_stop())

    def start_controller(self):
        self.is_running = True
        self.controller.start()

    def update_pid(self):
        try:
            kp = float(self.kp_entry.get())
            ki = float(self.ki_entry.get())
            kd = float(self.kd_entry.get())
            if not (0 <= kp <= 20): raise ValueError("Kp must be between 0 and 20")
            if not (0 <= ki <= 1): raise ValueError("Ki must be between 0 and 1")
            if not (0 <= kd <= 20): raise ValueError("Kd must be between 0 and 20")
            self.command_queue.put(("update_pid", (kp, ki, kd)))
            logger.info(f"PID updated: Kp={kp}, Ki={ki}, Kd={kd}")
        except ValueError as e:
            messagebox.showerror("Invalid Input", str(e))

    def update_gui(self):
        try:
            median_value, position_value = self.sensor_queue.get_nowait()
            if median_value is None or position_value is None:
                return  # Skip if sensor data is invalid
            self.intermediate_value.config(text=f"{median_value:.1f}")
            self.position_value_label.config(text=f"0x{position_value:04X}")
            active_bits = [i + 1 for i in range(16) if position_value & (1 << i)]
            location = sum(active_bits) / len(active_bits) if active_bits else 0
            self.location_value.config(text=f"{int(location)}")
            for i in range(16):
                bit_value = (position_value >> i) & 1
                fill = "#00AA00" if bit_value and i+1 in MIDDLE_SENSORS else "#33CC33" if bit_value else "lightgray"
                self.sensor_canvas.itemconfig(self.sensor_points[i][0], fill=fill)
        except queue.Empty:
            pass

        with self.controller.lock:
            self.status_var.set(f"State: {self.controller.state.name}")
            self.task1_status.set(f"Task 1: {'Running' if self.controller.state == AGVState.LINE_FOLLOW else 'Stopped'}")
            self.task1_error.set(f"PID Error: {self.controller.pid_error:.2f}")
            rpm1, cur1 = self.controller.motor_control.prev_fb_rpm[0], self.controller.motor_control.prev_fb_cur[0]
            rpm2, cur2 = self.controller.motor_control.prev_fb_rpm[1], self.controller.motor_control.prev_fb_cur[1]
            self.lbl_motor1.config(text=f"Motor 1: RPM {rpm1}, Current {cur1:.2f}A")
            self.lbl_motor2.config(text=f"Motor 2: RPM {rpm2}, Current {cur2:.2f}A")

        self.after(100, self.update_gui)

    def on_forward(self):
        self.command_queue.put(("set_state", AGVState.MANUAL_DRIVE))
        self.command_queue.put(("set_speed", (self.speed_var.get(), -self.speed_var.get())))

    def on_backward(self):
        self.command_queue.put(("set_state", AGVState.MANUAL_DRIVE))
        self.command_queue.put(("set_speed", (-self.speed_var.get(), self.speed_var.get())))

    def on_left(self):
        self.command_queue.put(("set_state", AGVState.MANUAL_DRIVE))
        self.command_queue.put(("set_speed", (-TURN_SPEED, -TURN_SPEED)))

    def on_right(self):
        self.command_queue.put(("set_state", AGVState.MANUAL_DRIVE))
        self.command_queue.put(("set_speed", (TURN_SPEED, TURN_SPEED)))

    def on_stop(self):
        self.command_queue.put(("set_state", AGVState.IDLE))
        self.command_queue.put(("set_speed", (0, 0)))

    def start_task1(self):
        self.command_queue.put(("set_state", AGVState.LINE_FOLLOW))

    def on_closing(self):
        self.is_running = False
        self.controller.stop()
        self.destroy()

class TextHandler(logging.Handler):
    def __init__(self, text_widget):
        super().__init__()
        self.text_widget = text_widget

    def emit(self, record):
        msg = self.format(record)
        self.text_widget.insert(tk.END, msg + '\n')
        self.text_widget.see(tk.END)

if __name__ == "__main__":
    app = AGVApp()
    app.mainloop()