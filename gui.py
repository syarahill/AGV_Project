import tkinter as tk
from tkinter import ttk, messagebox
import logging
import queue
import math
from controller import Controller, AGVState
from config import *

logger = logging.getLogger("AGV_GUI")

class AGVApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("AGV with Enhanced UWB Navigation and Magnetic Guide Sensor")
        self.geometry("1400x800")  # Increased size for diagnostics
        self.controller = Controller(enable_uwb=True)
        self.command_queue = self.controller.command_queue
        self.sensor_queue = self.controller.sensor_queue
        self.is_running = False
        
        # Grid map variables
        self.grid_size_x = 6  # 6 cells in X
        self.grid_size_y = 10  # 10 cells in Y  
        self.cell_size = 40   # Smaller cells for compact display
        self.agv_marker = None
        self.target_marker = None
        self.path_lines = []
        self.grid_cells = {}

        # GUI setup
        self.setup_gui()
        self.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.bind_keys()
        self.start_controller()
        self.after(30, self.update_gui)  # Faster updates: 30ms instead of 100ms

    def setup_gui(self):
        main_pane = ttk.PanedWindow(self, orient=tk.HORIZONTAL)
        main_pane.pack(fill="both", expand=True, padx=5, pady=5)

        # Left pane: Map and UWB Info
        left_frame = ttk.Frame(main_pane)
        main_pane.add(left_frame, weight=1)
        
        # Enhanced UWB Status Frame
        uwb_frame = ttk.LabelFrame(left_frame, text="Enhanced UWB Real-time Status")
        uwb_frame.pack(fill="x", padx=5, pady=5)
        
        # Position display
        pos_frame = ttk.Frame(uwb_frame)
        pos_frame.pack(fill="x", padx=5, pady=5)
        
        self.position_label = tk.Label(pos_frame, text="Position: X=0.00m, Y=0.00m", 
                                      font=("Arial", 12, "bold"), fg="blue")
        self.position_label.pack(side="left", padx=10)
        
        self.grid_label = tk.Label(pos_frame, text="Grid: (0, 0)", 
                                  font=("Arial", 12, "bold"), fg="green")
        self.grid_label.pack(side="left", padx=10)
        
        # Kalman filter status
        kalman_frame = ttk.Frame(uwb_frame)
        kalman_frame.pack(fill="x", padx=5, pady=2)
        
        self.kalman_status = tk.Label(kalman_frame, 
                                     text=f"Kalman Filter: {'Enabled' if UWB_KALMAN_ENABLED else 'Disabled'}", 
                                     font=("Arial", 10, "bold"), 
                                     fg="green" if UWB_KALMAN_ENABLED else "red")
        self.kalman_status.pack(side="left")
        
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
        
        # Anchor distances display with enhanced info
        anchor_frame = ttk.Frame(uwb_frame)
        anchor_frame.pack(fill="x", padx=5, pady=5)
        
        self.anchor_labels = {}
        self.anchor_quality_labels = {}
        anchor_info = [
            ("Anchor 0", "red"),
            ("Anchor 1", "green"),
            ("Anchor 2", "blue")
        ]
        
        for i, (name, color) in enumerate(anchor_info):
            frame = ttk.Frame(anchor_frame)
            frame.pack(side="left", padx=10)
            
            ttk.Label(frame, text=f"{name}:", font=("Arial", 10, "bold")).pack()
            
            distance_label = tk.Label(frame, text="0.00m", font=("Arial", 10), fg=color)
            distance_label.pack()
            self.anchor_labels[i] = distance_label
            
            quality_label = tk.Label(frame, text="●", font=("Arial", 12), fg="gray")
            quality_label.pack()
            self.anchor_quality_labels[i] = quality_label
        
        # Map Frame
        map_frame = ttk.LabelFrame(left_frame, text="Real-time Position Map")
        map_frame.pack(fill="both", expand=True, padx=5, pady=5)
        
        # Create grid map
        self.create_grid_map(map_frame)
        
        # Navigation Frame
        nav_frame = ttk.LabelFrame(left_frame, text="Navigation Control")
        nav_frame.pack(fill="x", padx=5, pady=5)
        
        nav_controls = ttk.Frame(nav_frame)
        nav_controls.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(nav_controls, text="Target X:", font=("Arial", 10)).grid(row=0, column=0, padx=5, sticky="e")
        self.target_x_entry = ttk.Entry(nav_controls, width=10)
        self.target_x_entry.grid(row=0, column=1, padx=5)
        self.target_x_entry.insert(0, "1.8")
        
        ttk.Label(nav_controls, text="Target Y:", font=("Arial", 10)).grid(row=0, column=2, padx=5, sticky="e")
        self.target_y_entry = ttk.Entry(nav_controls, width=10)
        self.target_y_entry.grid(row=0, column=3, padx=5)
        self.target_y_entry.insert(0, "3.0")
        
        self.navigate_btn = tk.Button(nav_controls, text="Navigate to Target", 
                                     command=self.navigate_to_target,
                                     bg="#27ae60", fg="white", font=("Arial", 10, "bold"),
                                     width=15)
        self.navigate_btn.grid(row=0, column=4, padx=10)
        
        self.nav_status = tk.Label(nav_frame, text="Navigation: Idle", 
                                  font=("Arial", 10), fg="black")
        self.nav_status.pack(pady=5)

        # Right pane: Controls and Sensor
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
        task_frame = ttk.LabelFrame(right_frame, text="Line Following Task")
        task_frame.pack(fill="x", padx=5, pady=5)
        
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

        # UWB Diagnostics Frame
        diag_frame = ttk.LabelFrame(right_frame, text="UWB Positioning Diagnostics")
        diag_frame.pack(fill="x", padx=5, pady=5)
        
        # Diagnostics display
        diag_display = ttk.Frame(diag_frame)
        diag_display.pack(fill="x", padx=5, pady=5)
        
        # Raw vs Filtered position
        if UWB_KALMAN_ENABLED:
            ttk.Label(diag_display, text="Raw Position:", font=("Arial", 9)).grid(row=0, column=0, sticky="w")
            self.raw_position_label = ttk.Label(diag_display, text="(0.000, 0.000)", font=("Arial", 9))
            self.raw_position_label.grid(row=0, column=1, padx=5, sticky="w")
            
            ttk.Label(diag_display, text="Filtered Position:", font=("Arial", 9)).grid(row=1, column=0, sticky="w")
            self.filtered_position_label = ttk.Label(diag_display, text="(0.000, 0.000)", font=("Arial", 9))
            self.filtered_position_label.grid(row=1, column=1, padx=5, sticky="w")
        
        # Error counters
        ttk.Label(diag_display, text="Position Jumps:", font=("Arial", 9)).grid(row=2, column=0, sticky="w")
        self.jump_count_label = ttk.Label(diag_display, text="0", font=("Arial", 9))
        self.jump_count_label.grid(row=2, column=1, padx=5, sticky="w")
        
        ttk.Label(diag_display, text="Trilateration Errors:", font=("Arial", 9)).grid(row=3, column=0, sticky="w")
        self.trilat_error_label = ttk.Label(diag_display, text="0", font=("Arial", 9))
        self.trilat_error_label.grid(row=3, column=1, padx=5, sticky="w")

        # Sensor Frame
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

    def create_grid_map(self, parent):
        """Create the visual grid map"""
        map_container = tk.Frame(parent, bg='white', relief=tk.SUNKEN, bd=2)
        map_container.pack(padx=10, pady=10)
        
        self.map_canvas = tk.Canvas(map_container, 
                                   width=self.grid_size_x * self.cell_size + 40,
                                   height=self.grid_size_y * self.cell_size + 40,
                                   bg='white', highlightthickness=0)
        self.map_canvas.pack()
        
        # Draw grid
        for i in range(self.grid_size_x + 1):
            x = i * self.cell_size + 20
            self.map_canvas.create_line(x, 20, x, self.grid_size_y * self.cell_size + 20,
                                       fill='#bdc3c7', width=1)
                                       
        for i in range(self.grid_size_y + 1):
            y = i * self.cell_size + 20
            self.map_canvas.create_line(20, y, self.grid_size_x * self.cell_size + 20, y,
                                       fill='#bdc3c7', width=1)
        
        # Add coordinate labels
        for i in range(self.grid_size_x):
            x = i * self.cell_size + self.cell_size/2 + 20
            self.map_canvas.create_text(x, 10, text=str(i), font=("Arial", 8))
                                       
        for i in range(self.grid_size_y):
            y = i * self.cell_size + self.cell_size/2 + 20
            self.map_canvas.create_text(10, y, text=str(self.grid_size_y - 1 - i), font=("Arial", 8))
        
        # Create grid cells
        for x in range(self.grid_size_x):
            for y in range(self.grid_size_y):
                x1 = x * self.cell_size + 21
                y1 = (self.grid_size_y - 1 - y) * self.cell_size + 21
                x2 = x1 + self.cell_size - 2
                y2 = y1 + self.cell_size - 2
                
                rect = self.map_canvas.create_rectangle(x1, y1, x2, y2,
                                                       fill='', outline='',
                                                       width=0)
                self.grid_cells[(x, y)] = rect
        
        # Create AGV marker
        self.agv_marker = self.map_canvas.create_oval(0, 0, 15, 15,
                                                     fill='#3498db', outline='#2980b9',
                                                     width=2)
        
        # Create direction indicator
        self.agv_direction = self.map_canvas.create_line(0, 0, 0, 0,
                                                        fill='#e74c3c', width=2,
                                                        arrow=tk.LAST)
        
        # Initially hide AGV
        self.map_canvas.itemconfig(self.agv_marker, state='hidden')
        self.map_canvas.itemconfig(self.agv_direction, state='hidden')
        
        # Mouse click event
        self.map_canvas.bind("<Button-1>", self.on_map_click)

    def update_agv_position(self, x, y, heading=90):
        """Update AGV position on the map"""
        # Fix coordinate conversion - scale properly to room size
        # Real room: 3.6m x 6.0m, GUI: 6 cells x 10 cells
        scale_x = self.grid_size_x / 3.6  # cells per meter X
        scale_y = self.grid_size_y / 6.0  # cells per meter Y
        
        canvas_x = (x * scale_x) * self.cell_size + 20
        canvas_y = (self.grid_size_y - (y * scale_y)) * self.cell_size + 20
        
        # Update AGV marker position
        self.map_canvas.coords(self.agv_marker,
                              canvas_x - 7.5, canvas_y - 7.5,
                              canvas_x + 7.5, canvas_y + 7.5)
        
        # Update direction indicator
        dx = 10 * math.cos(math.radians(heading))
        dy = -10 * math.sin(math.radians(heading))
        self.map_canvas.coords(self.agv_direction,
                              canvas_x, canvas_y,
                              canvas_x + dx, canvas_y + dy)
        
        # Show AGV
        self.map_canvas.itemconfig(self.agv_marker, state='normal')
        self.map_canvas.itemconfig(self.agv_direction, state='normal')
        
        # Fix grid cell calculation
        grid_x = int((x / 3.6) * self.grid_size_x)  # Scale to grid size
        grid_y = int((y / 6.0) * self.grid_size_y)  # Scale to grid size
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
        
        canvas_x = (x / 0.6) * self.cell_size + 20
        canvas_y = (self.grid_size_y - (y / 0.6)) * self.cell_size + 20
        
        self.target_marker = self.map_canvas.create_rectangle(
            canvas_x - 8, canvas_y - 8,
            canvas_x + 8, canvas_y + 8,
            fill='#e74c3c', outline='#c0392b', width=2
        )

    def on_map_click(self, event):
        """Handle click on map to set target"""
        # Fix coordinate conversion for map clicks
        grid_x = (event.x - 20) / self.cell_size
        grid_y = (event.y - 20) / self.cell_size
        
        x = (grid_x / self.grid_size_x) * 3.6  # Convert to real meters
        y = ((self.grid_size_y - grid_y) / self.grid_size_y) * 6.0  # Convert to real meters
        
        # Clamp to bounds
        x = max(0, min(3.6, x))
        y = max(0, min(6.0, y))
        
        # Update target entries
        self.target_x_entry.delete(0, tk.END)
        self.target_x_entry.insert(0, f"{x:.1f}")
        self.target_y_entry.delete(0, tk.END)
        self.target_y_entry.insert(0, f"{y:.1f}")
        
        # Update target marker
        self.update_target_position(x, y)

    def navigate_to_target(self):
        """Navigate to target position using UWB"""
        if not self.controller.uwb_enabled:
            messagebox.showwarning("UWB Not Available", 
                                  "UWB navigation is not available. Check connection.")
            return
            
        try:
            x = float(self.target_x_entry.get())
            y = float(self.target_y_entry.get())
            
            if not (0 <= x <= 3.6 and 0 <= y <= 6.0):
                raise ValueError("Coordinates must be within the 3.6x6.0m area")
            
            self.update_target_position(x, y)
            self.command_queue.put(("navigate_to", (x, y)))
            logger.info(f"Navigating to ({x:.1f}, {y:.1f}) with enhanced positioning")
            
        except ValueError as e:
            messagebox.showerror("Invalid Input", str(e))

    def reset_uwb_diagnostics(self):
        """Reset UWB diagnostics"""
        self.command_queue.put(("reset_uwb", None))
        logger.info("UWB diagnostics reset requested")

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
        # Update UWB position and enhanced diagnostics
        if self.controller.uwb_enabled and self.controller.uwb:
            x, y, z = self.controller.get_uwb_position()
            
            # Update position display
            self.position_label.config(text=f"Position: X={x:.3f}m, Y={y:.3f}m")
            
            # Update grid position
            if hasattr(self.controller.uwb, 'adj_grid'):
                grid_x, grid_y = self.controller.uwb.adj_grid(x, y)
                if grid_x is not None and grid_y is not None:
                    self.grid_label.config(text=f"Grid: ({grid_x}, {grid_y})")
            
            # Update anchor distances with quality indicators
            if hasattr(self.controller.uwb, 'get_anchor_distances'):
                distances = self.controller.uwb.get_anchor_distances()
                for i, label in self.anchor_labels.items():
                    if i in distances:
                        distance = distances[i]
                        label.config(text=f"{distance:.3f}m")
                        
                        # Update quality indicator
                        if UWB_MIN_DISTANCE <= distance <= UWB_MAX_DISTANCE:
                            self.anchor_quality_labels[i].config(fg="green")
                        else:
                            self.anchor_quality_labels[i].config(fg="red")
                    else:
                        self.anchor_quality_labels[i].config(fg="gray")
            
            # Update UWB diagnostics
            diagnostics = self.controller.get_uwb_diagnostics()
            
            # Update accuracy if Kalman is enabled
            if UWB_KALMAN_ENABLED and hasattr(self, 'accuracy_label'):
                avg_accuracy = diagnostics.get('average_accuracy', 0)
                self.accuracy_label.config(text=f"Avg Accuracy: {avg_accuracy:.3f}m")
            
            # Update position quality
            quality = diagnostics.get('position_quality', {})
            if quality:
                jump_count = quality.get('position_jump_count', 0)
                trilat_errors = quality.get('trilateration_error_count', 0)
                
                # Determine overall quality
                if jump_count > 3 or trilat_errors > 10:
                    quality_text = "Position Quality: Poor"
                    quality_color = "red"
                elif jump_count > 1 or trilat_errors > 5:
                    quality_text = "Position Quality: Fair"
                    quality_color = "orange"
                else:
                    quality_text = "Position Quality: Good"
                    quality_color = "green"
                
                self.position_quality_label.config(text=quality_text, fg=quality_color)
                
                # Update diagnostic labels
                self.jump_count_label.config(text=str(jump_count))
                self.trilat_error_label.config(text=str(trilat_errors))
                
                # Update raw vs filtered position display
                if UWB_KALMAN_ENABLED and hasattr(self, 'raw_position_label'):
                    raw_pos = quality.get('raw_position', (0, 0))
                    filtered_pos = quality.get('filtered_position', (0, 0))
                    self.raw_position_label.config(text=f"({raw_pos[0]:.3f}, {raw_pos[1]:.3f})")
                    self.filtered_position_label.config(text=f"({filtered_pos[0]:.3f}, {filtered_pos[1]:.3f})")
            
            # Update counters
            update_count = diagnostics.get('update_count', 0)
            self.updates_label.config(text=f"Updates: {update_count}")
            
            # Update AGV position on map
            if x > 0 or y > 0:  # Only update if we have valid position
                self.update_agv_position(x, y)
            
            # Update navigation status
            if self.controller.navigation:
                nav_status = self.controller.navigation.get_status()
                distance = nav_status['distance_to_target']
                if distance is not None:
                    self.nav_status.config(text=f"Navigation: {nav_status['state']} (Dist: {distance:.3f}m)")
                else:
                    self.nav_status.config(text=f"Navigation: {nav_status['state']}")
        
        # Update sensor data
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

        # Update status
        with self.controller.lock:
            self.status_var.set(f"State: {self.controller.state.name}")
            
            if self.controller.state == AGVState.LINE_FOLLOW:
                self.task1_status.config(text=f"Line Following: Active (Error: {self.controller.pid_error:.3f})")
                self.task1_btn.config(text="Stop Line Following", bg="#e74c3c")
            else:
                self.task1_status.config(text="Line Following: Stopped")
                self.task1_btn.config(text="Start Line Following", bg="#1ABC9C")
            
            self.task1_error.config(text=f"PID Error: {self.controller.pid_error:.3f}")
            
            rpm1, cur1 = self.controller.motor_control.prev_fb_rpm[0], self.controller.motor_control.prev_fb_cur[0]
            rpm2, cur2 = self.controller.motor_control.prev_fb_rpm[1], self.controller.motor_control.prev_fb_cur[1]
            self.lbl_motor1.config(text=f"Motor 1: RPM {rpm1}, Current {cur1:.2f}A")
            self.lbl_motor2.config(text=f"Motor 2: RPM {rpm2}, Current {cur2:.2f}A")

        self.after(30, self.update_gui)  # Faster updates: 30ms instead of 100ms

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