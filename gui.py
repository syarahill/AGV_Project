import tkinter as tk
from tkinter import ttk, messagebox
import logging
import queue
import math
import time
from controller import Controller, AGVState
from config import *

logger = logging.getLogger("AGV_GUI")

class AGVApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("AGV Control")
        self.geometry("1400x800")
        self.controller = Controller(enable_uwb=True)
        self.command_queue = self.controller.command_queue
        self.sensor_queue = self.controller.sensor_queue
        self.is_running = False
        
        # OPTIMIZED: Grid map variables
        self.grid_size_x = GRID_SIZE_X
        self.grid_size_y = GRID_SIZE_Y
        self.cell_size = 18
        self.agv_marker = None
        self.target_marker = None
        self.anchor_markers = {}
        self.path_lines = [] 
        self.grid_cells = {}
        
        # PERFORMANCE: GUI optimization variables
        self.gui_update_counter = 0
        self.last_agv_pos = (0, 0)
        self.position_update_threshold = GUI_POSITION_UPDATE_THRESHOLD
        self.agv_heading = 0
        self.last_uwb_pos = (0, 0)

        # GUI setup
        self.setup_gui()
        self.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.bind_keys()
        
        self.start_controller()
        self.after(GUI_UPDATE_RATE, self.update_gui)

    def setup_gui(self):
        main_pane = ttk.PanedWindow(self, orient=tk.HORIZONTAL)
        main_pane.pack(fill="both", expand=True, padx=5, pady=5)

        # FIXED: Left pane - simple frame without scrolling
        left_frame = ttk.Frame(main_pane)
        main_pane.add(left_frame, weight=1)
        
        # ENHANCED: Navigation Frame with Working Navigation Button - AT TOP!
        nav_frame = ttk.LabelFrame(left_frame, text="🎯 UWB Navigation Control")
        nav_frame.pack(fill="x", padx=5, pady=5)
        
        nav_controls = ttk.Frame(nav_frame)
        nav_controls.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(nav_controls, text="Target X:", font=("Arial", 10, "bold")).grid(row=0, column=0, padx=5, sticky="e")
        self.target_x_entry = ttk.Entry(nav_controls, width=10, font=("Arial", 10))
        self.target_x_entry.grid(row=0, column=1, padx=5)
        self.target_x_entry.insert(0, "2.0")  # Default to center
        
        ttk.Label(nav_controls, text="Target Y:", font=("Arial", 10, "bold")).grid(row=0, column=2, padx=5, sticky="e")
        self.target_y_entry = ttk.Entry(nav_controls, width=10, font=("Arial", 10))
        self.target_y_entry.grid(row=0, column=3, padx=5)
        self.target_y_entry.insert(0, "3.4")  # Default to center
        
        # ENHANCED: Navigation Button - This is the button you were looking for!
        self.navigate_btn = tk.Button(nav_controls, text="🎯 NAVIGATE TO TARGET", 
                                     command=self.navigate_to_target,
                                     bg="#27ae60", fg="white", font=("Arial", 12, "bold"),
                                     width=20, height=3, relief="raised", bd=4)
        self.navigate_btn.grid(row=0, column=4, padx=10, rowspan=2)
        
        # Quick navigation buttons
        quick_nav_frame = ttk.Frame(nav_controls)
        quick_nav_frame.grid(row=1, column=0, columnspan=4, pady=5)
        
        quick_positions = [
            ("🏠 Center", 2.2, 3.4),
            ("🔴 A0 Area", 3.8, 0.6),
            ("🟢 A1 Area", 3.8, 6.2),
            ("🔵 A2 Area", 0.2, 3.2)
        ]
        
        for i, (text, x, y) in enumerate(quick_positions):
            btn = tk.Button(quick_nav_frame, text=text, 
                           command=lambda x=x, y=y: self.quick_navigate(x, y),
                           bg="#3498db", fg="white", font=("Arial", 8), width=12)
            btn.pack(side="left", padx=2)
        
        self.nav_status = tk.Label(nav_frame, text="Navigation: Ready for target", 
                                  font=("Arial", 11, "bold"), fg="green")
        self.nav_status.pack(pady=5)
        
        # OPTIMIZED: Enhanced UWB Status Frame
        uwb_frame = ttk.LabelFrame(left_frame, text="Fast UWB Real-time Status")
        uwb_frame.pack(fill="x", padx=5, pady=5)
        
        # Position display with grid info
        pos_frame = ttk.Frame(uwb_frame)
        pos_frame.pack(fill="x", padx=5, pady=5)
        
        self.position_label = tk.Label(pos_frame, text="Position: X=0.00m, Y=0.00m", 
                                      font=("Arial", 12, "bold"), fg="blue")
        self.position_label.pack(side="left", padx=10)
        
        self.grid_label = tk.Label(pos_frame, text="Grid: (0, 0)", 
                                  font=("Arial", 12, "bold"), fg="green")
        self.grid_label.pack(side="left", padx=10)
        
        # Room info display
        room_info = tk.Label(pos_frame, 
                           text=f"Room: {ROOM_WIDTH}×{ROOM_HEIGHT}m ({GRID_SIZE_X}×{GRID_SIZE_Y} grid)", 
                           font=("Arial", 10), fg="gray")
        room_info.pack(side="left", padx=10)
        
        # Kalman filter status
        kalman_frame = ttk.Frame(uwb_frame)
        kalman_frame.pack(fill="x", padx=5, pady=2)
        
        self.kalman_status = tk.Label(kalman_frame, 
                                     text=f"Optimized Kalman: {'Enabled' if UWB_KALMAN_ENABLED else 'Disabled'}", 
                                     font=("Arial", 10, "bold"), 
                                     fg="green" if UWB_KALMAN_ENABLED else "red")
        self.kalman_status.pack(side="left")
        
        # Height compensation status
        height_status = tk.Label(kalman_frame, 
                               text=f"Height Comp: {UWB_HEIGHT_DIFFERENCE}m", 
                               font=("Arial", 10), fg="purple")
        height_status.pack(side="left", padx=20)
        
        if UWB_KALMAN_ENABLED:
            self.accuracy_label = tk.Label(kalman_frame, text="Avg Accuracy: 0.000m", 
                                          font=("Arial", 10), fg="purple")
            self.accuracy_label.pack(side="left", padx=20)
        
        # Position quality indicators
        quality_frame = ttk.Frame(uwb_frame)
        quality_frame.pack(fill="x", padx=5, pady=2)
        
        self.position_quality_label = tk.Label(quality_frame, text="Position Quality: Good", 
                                              font=("Arial", 10), fg="green")
        self.position_quality_label.pack(side="left")
        
        self.updates_label = tk.Label(quality_frame, text="Updates: 0", 
                                     font=("Arial", 10))
        self.updates_label.pack(side="left", padx=20)
        
        # UWB Reset button
        self.reset_uwb_btn = tk.Button(quality_frame, text="Reset UWB", 
                                      command=self.reset_uwb_diagnostics,
                                      bg="#e67e22", fg="white", font=("Arial", 9))
        self.reset_uwb_btn.pack(side="right", padx=5)
        
        # OPTIMIZED: Anchor distances display
        anchor_frame = ttk.LabelFrame(uwb_frame, text="Anchor Distances (3D → Ground)")
        anchor_frame.pack(fill="x", padx=5, pady=5)
        
        self.anchor_labels = {}
        self.anchor_ground_labels = {}
        self.anchor_quality_labels = {}
        
        # Create anchor info based on actual positions
        anchor_info = [
            (f"A0 @Grid(10,16)", "red", (4.0, 6.4)),
            (f"A1 @Grid(10,0)", "green", (4.0, 0.4)),
            (f"A2 @Grid(0,8)", "blue", (0.0, 3.2))
        ]
        
        for i, (name, color, pos) in enumerate(anchor_info):
            frame = ttk.Frame(anchor_frame)
            frame.pack(side="left", padx=10)
            
            ttk.Label(frame, text=f"{name}:", font=("Arial", 9, "bold")).pack()
            ttk.Label(frame, text=f"({pos[0]}, {pos[1]})", font=("Arial", 8), foreground="gray").pack()
            
            # 3D distance
            distance_label = tk.Label(frame, text="3D: 0.00m", font=("Arial", 9), fg=color)
            distance_label.pack()
            self.anchor_labels[i] = distance_label
            
            # Ground distance
            ground_label = tk.Label(frame, text="2D: 0.00m", font=("Arial", 9), fg=color)
            ground_label.pack()
            self.anchor_ground_labels[i] = ground_label
            
            # Quality indicator
            quality_label = tk.Label(frame, text="●", font=("Arial", 12), fg="gray")
            quality_label.pack()
            self.anchor_quality_labels[i] = quality_label
        
        # OPTIMIZED: Map Frame with FIXED HEIGHT so navigation doesn't get hidden
        map_frame = ttk.LabelFrame(left_frame, text=f"Fast Real-time Position Map ({GRID_SIZE_X}×{GRID_SIZE_Y} Grid)")
        map_frame.pack(fill="x", padx=5, pady=5)  # FIXED: Changed from fill="both", expand=True
        
        # Create grid map
        self.create_grid_map(map_frame)

        # Right pane: Controls and Sensor (keeping existing layout)
        right_frame = ttk.Frame(main_pane)
        main_pane.add(right_frame, weight=1)

        # Manual Control Frame
        control_frame = ttk.LabelFrame(right_frame, text="Manual Controls")
        control_frame.pack(fill="x", padx=5, pady=5)
        
        btn_frame = ttk.Frame(control_frame)
        btn_frame.pack(padx=10, pady=10)
        
        btn_style = {"width": 8, "height": 1, "bg": "#3498DB", "fg": "white", "font": ("Arial", 10)}
        tk.Button(btn_frame, text="Forward", command=self.on_forward, **btn_style).grid(row=0, column=1, pady=2)
        tk.Button(btn_frame, text="Left", command=self.on_left, **btn_style).grid(row=1, column=0, padx=2)
        tk.Button(btn_frame, text="Stop", command=self.on_stop, bg="#E74C3C", fg="white",
                  font=("Arial", 10), width=8, height=1).grid(row=1, column=1, pady=2)
        tk.Button(btn_frame, text="Right", command=self.on_right, **btn_style).grid(row=1, column=2, padx=2)
        tk.Button(btn_frame, text="Backward", command=self.on_backward, **btn_style).grid(row=2, column=1, pady=2)

        self.speed_var = tk.IntVar(value=int(MAX_SPEED))
        tk.Scale(control_frame, from_=0, to=30, orient=tk.HORIZONTAL, label="Speed (RPM)",
                 variable=self.speed_var, length=200).pack(pady=5)

        # Task Frame
        task_frame = ttk.LabelFrame(right_frame, text="Tasks")
        task_frame.pack(fill="x", padx=5, pady=5)
        
        # Line Following Task
        self.task1_btn = tk.Button(task_frame, text="Start Line Following", 
                                  command=self.start_task1,
                                  bg="#1ABC9C", fg="white", font=("Arial", 10, "bold"),
                                  width=20, height=2)
        self.task1_btn.pack(pady=5)
        
        self.task1_status = tk.Label(task_frame, text="Line Following: Stopped", 
                                    font=("Arial", 10))
        self.task1_status.pack(pady=2)
        
        self.task1_error = tk.Label(task_frame, text="PID Error: 0.00", 
                                   font=("Arial", 10))
        self.task1_error.pack(pady=2)

        # ENHANCED: Task 1 - Navigation to Grid (6,10) with detailed status
        separator = ttk.Separator(task_frame, orient='horizontal')
        separator.pack(fill='x', pady=10)
        
        self.task1_nav_btn = tk.Button(task_frame, text="🎯 Start Task 1 (Go to Grid 6,10)", 
                                      command=self.start_task1_navigation,
                                      bg="#9b59b6", fg="white", font=("Arial", 10, "bold"),
                                      width=25, height=2)
        self.task1_nav_btn.pack(pady=5)

        # Task 1 detailed status frame
        task1_status_frame = ttk.Frame(task_frame)
        task1_status_frame.pack(fill="x", padx=10, pady=5)
        
        self.task1_nav_status = tk.Label(task1_status_frame, text="Task 1: Ready", 
                                        font=("Arial", 10, "bold"))
        self.task1_nav_status.pack(pady=2)
        
        # Task 1 progress indicators
        self.task1_progress_frame = ttk.Frame(task1_status_frame)
        self.task1_progress_frame.pack(fill="x", pady=5)
        
        # Progress checkboxes
        self.task1_nav_complete_var = tk.BooleanVar()
        self.task1_nav_complete_cb = ttk.Checkbutton(self.task1_progress_frame, 
                                                     text="Navigation Complete", 
                                                     variable=self.task1_nav_complete_var,
                                                     state="disabled")
        self.task1_nav_complete_cb.pack(side="left", padx=5)
        
        self.task1_turn_complete_var = tk.BooleanVar()
        self.task1_turn_complete_cb = ttk.Checkbutton(self.task1_progress_frame, 
                                                      text="Left Turn Complete", 
                                                      variable=self.task1_turn_complete_var,
                                                      state="disabled")
        self.task1_turn_complete_cb.pack(side="left", padx=5)
        
        self.task1_line_active_var = tk.BooleanVar()
        self.task1_line_active_cb = ttk.Checkbutton(self.task1_progress_frame, 
                                                    text="Line Following Active", 
                                                    variable=self.task1_line_active_var,
                                                    state="disabled")
        self.task1_line_active_cb.pack(side="left", padx=5)

        # Add Trajectory Logging buttons
        traj_frame = ttk.Frame(task_frame)
        traj_frame.pack(fill="x", padx=10, pady=5)

        self.start_log_btn = tk.Button(traj_frame, text="Start Logging", 
                                     command=self.start_trajectory_logging,
                                     bg="#3498db", fg="white", font=("Arial", 9))
        self.start_log_btn.pack(side="left", padx=5)

        self.stop_log_btn = tk.Button(traj_frame, text="Stop Logging", 
                                    command=self.stop_trajectory_logging,
                                    bg="#e74c3c", fg="white", font=("Arial", 9))
        self.stop_log_btn.pack(side="left", padx=5)

        self.traj_status = tk.Label(traj_frame, text="Logging: OFF", font=("Arial", 9))
        self.traj_status.pack(side="left", padx=10)

        # Battery Status Frame
        battery_frame = ttk.LabelFrame(right_frame, text="🔋 Battery Status")
        battery_frame.pack(fill="x", padx=5, pady=5)
        
        battery_grid = ttk.Frame(battery_frame)
        battery_grid.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(battery_grid, text="Voltage:", font=("Arial", 10, "bold")).grid(row=0, column=0, sticky="w")
        self.battery_voltage_label = ttk.Label(battery_grid, text="0.00 V", font=("Arial", 10))
        self.battery_voltage_label.grid(row=0, column=1, padx=10, sticky="w")
        
        ttk.Label(battery_grid, text="Charge:", font=("Arial", 10, "bold")).grid(row=0, column=2, sticky="w")
        self.battery_percentage_label = ttk.Label(battery_grid, text="0%", font=("Arial", 10))
        self.battery_percentage_label.grid(row=0, column=3, padx=10, sticky="w")
        
        ttk.Label(battery_grid, text="Status:", font=("Arial", 10, "bold")).grid(row=0, column=4, sticky="w")
        self.battery_status_label = ttk.Label(battery_grid, text="UNKNOWN", font=("Arial", 10))
        self.battery_status_label.grid(row=0, column=5, padx=10, sticky="w")
        
        # Charging indicator
        self.charging_indicator = tk.Label(battery_frame, text="⚡", font=("Arial", 20), fg="gray")
        self.charging_indicator.pack(pady=5)

        # PID settings
        pid_frame = ttk.Frame(task_frame)
        pid_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(pid_frame, text="Kp:", font=("Arial", 9)).grid(row=0, column=0, padx=2, sticky="e")
        self.kp_entry = ttk.Entry(pid_frame, width=8)
        self.kp_entry.grid(row=0, column=1, padx=2)
        self.kp_entry.insert(0, str(INITIAL_PID_KP))
        
        ttk.Label(pid_frame, text="Ki:", font=("Arial", 9)).grid(row=0, column=2, padx=2, sticky="e")
        self.ki_entry = ttk.Entry(pid_frame, width=8)
        self.ki_entry.grid(row=0, column=3, padx=2)
        self.ki_entry.insert(0, str(INITIAL_PID_KI))
        
        ttk.Label(pid_frame, text="Kd:", font=("Arial", 9)).grid(row=0, column=4, padx=2, sticky="e")
        self.kd_entry = ttk.Entry(pid_frame, width=8)
        self.kd_entry.grid(row=0, column=5, padx=2)
        self.kd_entry.insert(0, str(INITIAL_PID_KD))
        
        tk.Button(pid_frame, text="Update PID", command=self.update_pid,
                  bg="#F1C40F", fg="black", font=("Arial", 9)).grid(row=0, column=6, padx=5)

        # OPTIMIZED: UWB Diagnostics Frame (simplified)
        diag_frame = ttk.LabelFrame(right_frame, text="Fast UWB Diagnostics")
        diag_frame.pack(fill="x", padx=5, pady=5)
        
        # Simplified diagnostics display
        diag_display = ttk.Frame(diag_frame)
        diag_display.pack(fill="x", padx=5, pady=5)
        
        # Performance indicator
        ttk.Label(diag_display, text="Performance:", font=("Arial", 9, "bold")).grid(row=0, column=0, sticky="w")
        self.performance_label = ttk.Label(diag_display, text="Optimized", font=("Arial", 9), foreground="green")
        self.performance_label.grid(row=0, column=1, padx=5, sticky="w")
        
        # Update rate display
        ttk.Label(diag_display, text="Update Rate:", font=("Arial", 9)).grid(row=1, column=0, sticky="w")
        self.update_rate_label = ttk.Label(diag_display, text="0 Hz", font=("Arial", 9))
        self.update_rate_label.grid(row=1, column=1, padx=5, sticky="w")

        # OPTIMIZED: Sensor Frame (keeping functionality)
        sensor_frame = ttk.LabelFrame(right_frame, text="Magnetic Sensor Data")
        sensor_frame.pack(fill="both", expand=True, padx=5, pady=5)

        data_display = ttk.Frame(sensor_frame)
        data_display.pack(fill="x", padx=5, pady=5)
        
        ttk.Label(data_display, text="Median Value:", font=("Arial", 9)).grid(row=0, column=0, sticky="w")
        self.intermediate_value = ttk.Label(data_display, text="0", font=("Arial", 9, "bold"))
        self.intermediate_value.grid(row=0, column=1, padx=5, sticky="w")
        
        ttk.Label(data_display, text="Position (Hex):", font=("Arial", 9)).grid(row=1, column=0, sticky="w")
        self.position_value_label = ttk.Label(data_display, text="0x0000", font=("Arial", 9, "bold"))
        self.position_value_label.grid(row=1, column=1, padx=5, sticky="w")

        # Sensor visualization
        ttk.Label(sensor_frame, text="Sensors: [1-8=RIGHT] | [9-16=LEFT]", font=("Arial", 8)).pack(pady=2)
        self.sensor_canvas = tk.Canvas(sensor_frame, width=400, height=60, bg="white")
        self.sensor_canvas.pack(fill="x", padx=5, pady=5)
        
        self.sensor_points = []
        for i in range(16):
            x = 20 + i * 23
            y = 30
            circle = self.sensor_canvas.create_oval(x-8, y-8, x+8, y+8, fill="white", outline="black")
            text = self.sensor_canvas.create_text(x, y, text=str(i+1), font=("Arial", 7))
            self.sensor_points.append((circle, text))

        # Status Frame
        status_frame = ttk.Frame(right_frame)
        status_frame.pack(fill="x", padx=5, pady=5)
        
        self.status_var = tk.StringVar(value="Connecting...")
        self.status_label = tk.Label(status_frame, textvariable=self.status_var, 
                                    font=("Arial", 10), fg="red")
        self.status_label.pack()
        
        # Motor feedback
        feedback_frame = ttk.LabelFrame(right_frame, text="Motor Feedback")
        feedback_frame.pack(fill="x", padx=5, pady=5)
        
        self.lbl_motor1 = tk.Label(feedback_frame, text="Motor 1: Unknown", font=("Arial", 9))
        self.lbl_motor1.pack(anchor="w", padx=5)
        self.lbl_motor2 = tk.Label(feedback_frame, text="Motor 2: Unknown", font=("Arial", 9))
        self.lbl_motor2.pack(anchor="w", padx=5)

        # Log text
        self.log_text = tk.Text(right_frame, height=6, width=40, font=("Arial", 8))
        self.log_text.pack(fill="x", padx=5, pady=5)
        self.log_handler = TextHandler(self.log_text)
        logger.addHandler(self.log_handler)

    def start_trajectory_logging(self):
        self.command_queue.put(("start_trajectory_logging", None))
        self.traj_status.config(text="Logging: ON", fg="green")
        logger.info("Trajectory logging started")

    def stop_trajectory_logging(self):
        self.command_queue.put(("stop_trajectory_logging", None))
        self.traj_status.config(text="Logging: OFF", fg="black")
        logger.info("Trajectory logging stopped")

    def create_grid_map(self, parent):
        """OPTIMIZED: Create compact visual grid map (ORIGINAL COORDINATES - CORRECT)"""
        map_container = tk.Frame(parent, bg='white', relief=tk.SUNKEN, bd=2)
        map_container.pack(padx=5, pady=5)
        
        # SMALLER: Calculate compact canvas size
        canvas_width = self.grid_size_x * self.cell_size + 30
        canvas_height = self.grid_size_y * self.cell_size + 30
        
        self.map_canvas = tk.Canvas(map_container, 
                                   width=canvas_width,
                                   height=canvas_height,
                                   bg='white', highlightthickness=0)
        self.map_canvas.pack()
        
        # Draw grid lines
        for i in range(self.grid_size_x + 1):
            x = i * self.cell_size + 15
            self.map_canvas.create_line(x, 15, x, self.grid_size_y * self.cell_size + 15,
                                       fill='#bdc3c7', width=1)
                                       
        for i in range(self.grid_size_y + 1):
            y = i * self.cell_size + 15
            self.map_canvas.create_line(15, y, self.grid_size_x * self.cell_size + 15, y,
                                       fill='#bdc3c7', width=1)
        
        # Add coordinate labels (ORIGINAL WITH Y FLIP - CORRECT)
        for i in range(0, self.grid_size_x, 3):
            x = i * self.cell_size + self.cell_size/2 + 15
            self.map_canvas.create_text(x, 8, text=str(i), font=("Arial", 7))
                                       
        for i in range(0, self.grid_size_y, 3):
            y = i * self.cell_size + self.cell_size/2 + 15
            self.map_canvas.create_text(8, self.grid_size_y * self.cell_size - y + 15, 
                                       text=str(i), font=("Arial", 7))  # ORIGINAL: Y flip
        
        # Create grid cells (ORIGINAL WITH Y FLIP - CORRECT)
        for x in range(self.grid_size_x):
            for y in range(self.grid_size_y):
                x1 = x * self.cell_size + 16
                y1 = (self.grid_size_y - 1 - y) * self.cell_size + 16  # ORIGINAL: Y flip
                x2 = x1 + self.cell_size - 2
                y2 = y1 + self.cell_size - 2
                
                rect = self.map_canvas.create_rectangle(x1, y1, x2, y2,
                                                       fill='', outline='',
                                                       width=0)
                self.grid_cells[(x, y)] = rect
        
        # Create anchor markers at CORRECT positions (ORIGINAL WITH Y FLIP - CORRECT)
        anchor_positions = [
            (10, 16, "A0", "red"),
            (10, 1, "A1", "green"),
            (0, 8, "A2", "blue")
        ]
        
        for grid_x, grid_y, label, color in anchor_positions:
            canvas_x = grid_x * self.cell_size + self.cell_size/2 + 15
            canvas_y = (self.grid_size_y - 1 - grid_y) * self.cell_size + self.cell_size/2 + 15  # ORIGINAL: Y flip
            
            anchor = self.map_canvas.create_oval(canvas_x-6, canvas_y-6, canvas_x+6, canvas_y+6,
                                               fill=color, outline='black', width=2)
            anchor_text = self.map_canvas.create_text(canvas_x, canvas_y-12, text=label, 
                                                    font=("Arial", 8, "bold"), fill=color)
            
            self.anchor_markers[label] = (anchor, anchor_text)
        
        # Highlight target grid (6,10) for Task 1 (ORIGINAL WITH Y FLIP - CORRECT)
        target_grid_x, target_grid_y = TASK1_TARGET_GRID
        if 0 <= target_grid_x < self.grid_size_x and 0 <= target_grid_y < self.grid_size_y:
            canvas_x = target_grid_x * self.cell_size + self.cell_size/2 + 15
            canvas_y = (self.grid_size_y - 1 - target_grid_y) * self.cell_size + self.cell_size/2 + 15  # ORIGINAL: Y flip
            
            target_marker = self.map_canvas.create_rectangle(
                canvas_x - 8, canvas_y - 8, canvas_x + 8, canvas_y + 8,
                fill='#8e44ad', outline='#9b59b6', width=2
            )
            target_text = self.map_canvas.create_text(canvas_x, canvas_y + 15, 
                                                    text="Task1", font=("Arial", 8, "bold"), 
                                                    fill='#8e44ad')
        
        # Create AGV marker
        self.agv_marker = self.map_canvas.create_oval(0, 0, 12, 12,
                                                     fill='#3498db', outline='#2980b9',
                                                     width=2)
        
        # Create direction indicator
        self.agv_direction = self.map_canvas.create_line(0, 0, 0, 0,
                                                        fill='#e74c3c', width=2,
                                                        arrow=tk.LAST)
        
        # Initially hide AGV
        self.map_canvas.itemconfig(self.agv_marker, state='hidden')
        self.map_canvas.itemconfig(self.agv_direction, state='hidden')
        
        # Mouse click event for easy target setting
        self.map_canvas.bind("<Button-1>", self.on_map_click)

    def real_to_canvas_coords(self, real_x, real_y):
        """Convert real-world coordinates to canvas coordinates (ORIGINAL WITH Y FLIP - CORRECT for map elements)"""
        grid_x = real_x / CELL_SIZE
        grid_y = real_y / CELL_SIZE
        
        canvas_x = grid_x * self.cell_size + 15
        canvas_y = (self.grid_size_y - grid_y) * self.cell_size + 15  # ORIGINAL: Y flip
        
        return canvas_x, canvas_y

    def agv_real_to_canvas_coords(self, real_x, real_y):
        """FIXED: Convert AGV real-world coordinates to canvas coordinates (NO Y FLIP for AGV)"""
        grid_x = real_x / CELL_SIZE
        grid_y = real_y / CELL_SIZE
        
        canvas_x = grid_x * self.cell_size + 15
        canvas_y = (self.grid_size_y - 1 - grid_y) * self.cell_size + 15  # Corrected Y flip
        
        return canvas_x, canvas_y

    def canvas_to_real_coords(self, canvas_x, canvas_y):
        """Convert canvas coordinates to real-world coordinates (ORIGINAL WITH Y FLIP - CORRECT)"""
        grid_x = (canvas_x - 15) / self.cell_size
        grid_y = self.grid_size_y - (canvas_y - 15) / self.cell_size  # ORIGINAL: Y flip
        
        real_x = grid_x * CELL_SIZE
        real_y = grid_y * CELL_SIZE
        
        return real_x, real_y

    def draw_planned_path(self, path_lines):
        """Draw planned path as green lines on map"""
        # Clear previous path lines
        for line in getattr(self, 'path_lines', []):
            self.map_canvas.delete(line)
        self.path_lines = []
        
        # Draw new path lines using AGV coordinate system (no Y flip)
        for from_pos, to_pos in path_lines:
            from_canvas = self.agv_real_to_canvas_coords(from_pos[0], from_pos[1])
            to_canvas = self.agv_real_to_canvas_coords(to_pos[0], to_pos[1])
            
            line = self.map_canvas.create_line(
                from_canvas[0], from_canvas[1],
                to_canvas[0], to_canvas[1],
                fill='#27ae60', width=3, tags='path'
            )
            self.path_lines.append(line)

    def update_agv_position(self, x, y, heading=90):
        """FIXED: Update AGV position with correct coordinates (NO Y FLIP for AGV)"""
        # Only update if significant movement
        dx = x - self.last_agv_pos[0]
        dy = y - self.last_agv_pos[1]
        if (dx**2 + dy**2) < self.position_update_threshold**2:
            return  # Skip update if movement too small
        
        self.last_agv_pos = (x, y)
        
        # FIXED: Use AGV-specific coordinate conversion (no Y flip)
        canvas_x, canvas_y = self.agv_real_to_canvas_coords(x, y)
        
        # Update AGV marker position
        self.map_canvas.coords(self.agv_marker,
                              canvas_x - 6, canvas_y - 6,
                              canvas_x + 6, canvas_y + 6)
        
        # Update direction indicator
        dx = 15 * math.cos(math.radians(heading))
        dy = -15 * math.sin(math.radians(heading))
        self.map_canvas.coords(self.agv_direction,
                              canvas_x, canvas_y,
                              canvas_x + dx, canvas_y + dy)
        
        # Show AGV
        self.map_canvas.itemconfig(self.agv_marker, state='normal')
        self.map_canvas.itemconfig(self.agv_direction, state='normal')
        
        # FIXED: Highlight current grid cell with correct coordinates (no Y flip)
        grid_x = int(x / CELL_SIZE)
        grid_y = int(y / CELL_SIZE)
        if 0 <= grid_x < self.grid_size_x and 0 <= grid_y < self.grid_size_y:
            for coord, rect in self.grid_cells.items():
                if coord == (grid_x, grid_y):
                    self.map_canvas.itemconfig(rect, fill='#e8f6f3', outline='#27ae60', width=2)
                else:
                    self.map_canvas.itemconfig(rect, fill='', outline='', width=0)

    def update_target_position(self, x, y):
        """Update target position on the map"""
        if self.target_marker:
            self.map_canvas.delete(self.target_marker)
        
        canvas_x, canvas_y = self.real_to_canvas_coords(x, y)
        
        self.target_marker = self.map_canvas.create_rectangle(
            canvas_x - 8, canvas_y - 8,
            canvas_x + 8, canvas_y + 8,
            fill='#e74c3c', outline='#c0392b', width=2
        )

    def on_map_click(self, event):
        """Handle click on map to set target"""
        real_x, real_y = self.canvas_to_real_coords(event.x, event.y)
        
        # Clamp to room bounds
        real_x = max(0, min(ROOM_WIDTH, real_x))
        real_y = max(0, min(ROOM_HEIGHT, real_y))
        
        # Update target entries
        self.target_x_entry.delete(0, tk.END)
        self.target_x_entry.insert(0, f"{real_x:.1f}")
        self.target_y_entry.delete(0, tk.END)
        self.target_y_entry.insert(0, f"{real_y:.1f}")
        
        # Update target marker
        self.update_target_position(real_x, real_y)

    def navigate_to_target(self):
        """Navigate to target position using fast UWB"""
        if not self.controller.uwb_enabled:
            messagebox.showwarning("UWB Not Available", 
                                  "UWB navigation is not available. Check UWB connection and try again.")
            return
            
        try:
            x = float(self.target_x_entry.get())
            y = float(self.target_y_entry.get())
            
            if not (0 <= x <= ROOM_WIDTH and 0 <= y <= ROOM_HEIGHT):
                raise ValueError(f"Coordinates must be within the {ROOM_WIDTH}×{ROOM_HEIGHT}m room area")
            
            # Update visual target
            self.update_target_position(x, y)
            
            # Send navigation command to controller
            self.command_queue.put(("navigate_to", (x, y)))
            
            # Update UI feedback
            self.nav_status.config(text=f"🎯 Navigating to ({x:.1f}, {y:.1f})...", fg="blue")
            self.navigate_btn.config(text="🔄 Navigating...", bg="#e67e22")
            
            logger.info(f"Navigation started to ({x:.1f}, {y:.1f}) with fast positioning")
            
        except ValueError as e:
            messagebox.showerror("Invalid Input", str(e))

    def quick_navigate(self, x, y):
        """Quick navigation to preset positions"""
        self.target_x_entry.delete(0, tk.END)
        self.target_x_entry.insert(0, f"{x:.1f}")
        self.target_y_entry.delete(0, tk.END)
        self.target_y_entry.insert(0, f"{y:.1f}")
        self.navigate_to_target()

    def start_task1_navigation(self):
        """Start Task 1 - Navigate to grid (6,10)"""
        if not self.controller.uwb_enabled:
            messagebox.showwarning("UWB Not Available", 
                                  "UWB navigation is required for Task 1. Check UWB connection.")
            return
        
        # Reset progress indicators
        self.task1_nav_complete_var.set(False)
        self.task1_turn_complete_var.set(False)
        self.task1_line_active_var.set(False)
        
        # FIXED: Send correct command for Task 1
        self.command_queue.put(("start_task1", None))
        logger.info("=== TASK 1 STARTED ===")

    def reset_uwb_diagnostics(self):
        """Reset UWB diagnostics"""
        self.command_queue.put(("reset_uwb", None))
        logger.info("UWB diagnostics reset requested")

    def update_anchor_distances(self):
        """OPTIMIZED: Separate method for anchor distance updates"""
        if hasattr(self.controller.uwb, 'get_anchor_distances'):
            distances_3d = self.controller.uwb.get_anchor_distances()
            distances_2d = self.controller.uwb.get_anchor_distances_2d()
            
            for i, label in self.anchor_labels.items():
                if i in distances_3d:
                    distance_3d = distances_3d[i]
                    distance_2d = distances_2d.get(i, 0)
                    
                    label.config(text=f"3D: {distance_3d:.3f}m")
                    self.anchor_ground_labels[i].config(text=f"2D: {distance_2d:.3f}m")
                    
                    # Update quality indicator
                    if 0.1 <= distance_3d <= 25.0:
                        self.anchor_quality_labels[i].config(fg="green")
                    else:
                        self.anchor_quality_labels[i].config(fg="red")

    def update_diagnostics(self):
        """OPTIMIZED: Separate method for diagnostics updates"""
        diagnostics = self.controller.get_uwb_diagnostics()
        
        if UWB_KALMAN_ENABLED and hasattr(self, 'accuracy_label'):
            avg_accuracy = diagnostics.get('average_accuracy', 0)
            self.accuracy_label.config(text=f"Avg Accuracy: {avg_accuracy:.3f}m")
        
        update_count = diagnostics.get('update_count', 0)
        self.updates_label.config(text=f"Updates: {update_count}")
        
        # Update rate calculation
        if hasattr(self, 'last_update_count'):
            rate = (update_count - self.last_update_count) * (1000 / GUI_UPDATE_RATE / GUI_HEAVY_UPDATE_INTERVAL)
            self.update_rate_label.config(text=f"{rate:.1f} Hz")
        self.last_update_count = update_count

    def update_sensor_display(self):
        """OPTIMIZED: Update sensor display"""
        try:
            median_value, position_value = self.sensor_queue.get_nowait()
            if median_value is not None and position_value is not None:
                self.intermediate_value.config(text=f"{median_value:.1f}")
                self.position_value_label.config(text=f"0x{position_value:04X}")
                
                # Update sensor visualization
                for i in range(16):
                    bit_value = not ((position_value >> i) & 1)
                    if bit_value:
                        if i+1 in CENTER_SENSORS:
                            fill = '#00AA00'
                        elif i+1 in LOW_SENSORS:
                            fill = '#FF6B6B'
                        else:
                            fill = '#6BB6FF'
                    else:
                        fill = 'lightgray'
                    self.sensor_canvas.itemconfig(self.sensor_points[i][0], fill=fill)
        except queue.Empty:
            pass

    def update_battery_display(self):
        """Update battery status display"""
        voltage = self.controller.battery_voltage
        percentage = self.controller.battery_percentage
        status = self.controller.battery_status
        
        # Override status if charging is detected
        if self.controller.state == AGVState.CHARGING and self.controller.charging_detected:
            status = "Charging"
        elif self.controller.state == AGVState.CHARGING_RETRY:
            status = "Charging Retry"
        elif self.controller.state == AGVState.ERROR and self.controller.charging_retry_count >= MAX_CHARGING_RETRIES:
            status = "Not Charging"
        
        self.battery_voltage_label.config(text=f"{voltage:.2f} V")
        self.battery_percentage_label.config(text=f"{percentage}%")
        self.battery_status_label.config(text=status)
        
        # Update charging indicator
        if status == "Charging":
            self.charging_indicator.config(fg="gold")
        elif status == "Not Charging":
            self.charging_indicator.config(fg="red")
        else:
            self.charging_indicator.config(fg="gray")

    def update_status_display(self):
        """OPTIMIZED: Update status display with Task 1 progress"""
        with self.controller.lock:
            self.status_var.set(f"State: {self.controller.state.name}")
            
            if self.controller.state == AGVState.LINE_FOLLOW:
                self.task1_status.config(text=f"Line Following: Active (Error: {self.controller.line_following.pid_error:.3f})")
                self.task1_btn.config(text="Stop Line Following", bg="#e74c3c")
            else:
                self.task1_status.config(text="Line Following: Stopped")
                self.task1_btn.config(text="Start Line Following", bg="#1ABC9C")
            
            self.task1_error.config(text=f"PID Error: {self.controller.line_following.pid_error:.3f}")
            
            # Update Task 1 Navigation status with enhanced progress tracking
            if self.controller.state == AGVState.TASK1_NAVIGATION:
                self.task1_nav_btn.config(text="🔄 Task 1: Navigating...", bg="#e67e22")
                self.task1_nav_status.config(text=f"Task 1: Moving to grid {TASK1_TARGET_GRID}", fg="blue")
                
                # Check if navigation completed
                if hasattr(self.controller, 'task1_nav_complete') and self.controller.task1_nav_complete:
                    self.task1_nav_complete_var.set(True)
                    
            elif self.controller.state == AGVState.TURNING_LEFT:
                self.task1_nav_btn.config(text="🔄 Task 1: Turning Left...", bg="#e67e22")
                self.task1_nav_status.config(text="Task 1: Executing 90° left turn", fg="orange")
                
                # Navigation must be complete to be turning
                self.task1_nav_complete_var.set(True)
                
                # Check if turn completed
                if hasattr(self.controller, 'task1_turn_complete') and self.controller.task1_turn_complete:
                    self.task1_turn_complete_var.set(True)
                    
            elif self.controller.state == AGVState.LINE_FOLLOW:
                # Check if this is part of Task 1
                if (hasattr(self.controller, 'task1_nav_complete') and self.controller.task1_nav_complete and
                    hasattr(self.controller, 'task1_turn_complete') and self.controller.task1_turn_complete):
                    self.task1_nav_status.config(text="Task 1: Line following active", fg="green")
                    self.task1_line_active_var.set(True)
                    self.task1_nav_btn.config(text="✓ Task 1 Complete", bg="#27ae60")
                    
            elif self.controller.state == AGVState.CHARGING:
                # If all Task 1 steps complete and now charging
                if (hasattr(self.controller, 'task1_nav_complete') and self.controller.task1_nav_complete and
                    hasattr(self.controller, 'task1_turn_complete') and self.controller.task1_turn_complete):
                    self.task1_nav_status.config(text="Task 1: Complete - Charging", fg="gold")
                    self.task1_nav_btn.config(text="✓ Task 1 Finished", bg="#27ae60")
            
            # Add charging retry status
            elif self.controller.state == AGVState.CHARGING_RETRY:
                elapsed = time.time() - self.controller.charging_retry_start
                remaining = max(0, CHARGING_RETRY_DURATION - elapsed)
                self.task1_nav_status.config(
                    text=f"Charging retry #{self.controller.charging_retry_count} - {remaining:.1f}s remaining",
                    fg="orange"
                )
            
            else:
                # Reset Task 1 button if idle or other state
                if self.controller.state == AGVState.IDLE:
                    self.task1_nav_btn.config(text="🎯 Start Task 1 (Go to Grid 6,10)", bg="#9b59b6")
                    self.task1_nav_status.config(text="Task 1: Ready", fg="black")
            
            # Update navigation button based on state
            if self.controller.state == AGVState.UWB_NAVIGATION:
                self.navigate_btn.config(text="🔄 Navigating...", bg="#e67e22")
                self.nav_status.config(text=f"🎯 Navigating to target...", fg="blue")
            else:
                self.navigate_btn.config(text="🎯 NAVIGATE TO TARGET", bg="#27ae60")
                if self.controller.state == AGVState.IDLE:
                    self.nav_status.config(text="Navigation: Ready for target", fg="green")
                elif self.controller.state == AGVState.LINE_FOLLOW:
                    self.nav_status.config(text="Navigation: Line following mode", fg="purple")
            
            rpm1, cur1 = self.controller.motor_control.prev_fb_rpm[0], self.controller.motor_control.prev_fb_cur[0]
            rpm2, cur2 = self.controller.motor_control.prev_fb_rpm[1], self.controller.motor_control.prev_fb_cur[1]
            self.lbl_motor1.config(text=f"Motor 1: RPM {rpm1}, Current {cur1:.2f}A")
            self.lbl_motor2.config(text=f"Motor 2: RPM {rpm2}, Current {cur2:.2f}A")

    def update_gui(self):
        """OPTIMIZED: Main GUI update with performance optimization"""
        self.gui_update_counter += 1
        
        # UWB updates every cycle
        if self.controller.uwb_enabled and self.controller.uwb:
            x, y, z = self.controller.get_uwb_position()
            
            # Calculate heading if we have previous position
            if hasattr(self, 'last_uwb_pos'):
                last_x, last_y = self.last_uwb_pos
                dx = x - last_x
                dy = y - last_y
                distance = math.sqrt(dx**2 + dy**2)
                
                # Only update heading if we've moved significantly
                if distance > 0.1:  # 10cm movement
                    heading = math.degrees(math.atan2(dx, dy))
                    if heading < 0:
                        heading += 360
                    self.agv_heading = heading
            
            self.last_uwb_pos = (x, y)
            
            # Update position display every cycle
            self.position_label.config(text=f"Position: X={x:.3f}m, Y={y:.3f}m")
            
            # Grid position every cycle
            grid_x = int(x / CELL_SIZE)
            grid_y = int(y / CELL_SIZE)
            self.grid_label.config(text=f"Grid: ({grid_x}, {grid_y})")
            
            # Update map position (with movement threshold for performance)
            if x > 0 or y > 0:
                self.update_agv_position(x, y, self.agv_heading)
        
        # Heavy operations less frequently for better performance
        if self.gui_update_counter % GUI_HEAVY_UPDATE_INTERVAL == 0:
            self.update_anchor_distances()
            self.update_diagnostics()
        
        # Sensor data every cycle
        self.update_sensor_display()
        
        # Battery data every cycle
        self.update_battery_display()
        
        # Status updates every cycle
        self.update_status_display()
        
        # Continue optimized updates
        self.after(GUI_UPDATE_RATE, self.update_gui)

    def bind_keys(self):
        self.bind("<Up>", lambda e: self.on_forward())
        self.bind("<Down>", lambda e: self.on_backward())
        self.bind("<Left>", lambda e: self.on_left())
        self.bind("<Right>", lambda e: self.on_right())
        self.bind("<space>", lambda e: self.on_stop())
        # ADDED: Keyboard shortcut for navigation
        self.bind("<Return>", lambda e: self.navigate_to_target())  # Enter key to navigate
        self.bind("<Control-g>", lambda e: self.navigate_to_target())  # Ctrl+G to navigate

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

    def on_forward(self):
        self.command_queue.put(("set_state", AGVState.MANUAL_DRIVE))
        speed = self.speed_var.get()
        self.command_queue.put(("set_speed", (speed, -speed)))

    def on_backward(self):
        self.command_queue.put(("set_state", AGVState.MANUAL_DRIVE))
        speed = self.speed_var.get()
        self.command_queue.put(("set_speed", (-speed, speed)))

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
        if self.controller.state == AGVState.LINE_FOLLOW:
            self.on_stop()
        else:
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