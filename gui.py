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
        self.title("AGV Control - Unified System")
        self.geometry("1800x900")  # FIXED: Optimized for 1920x1080 screen
        self.controller = Controller(enable_uwb=True)
        self.command_queue = self.controller.command_queue
        self.sensor_queue = self.controller.sensor_queue
        self.is_running = False
        
        # Grid map variables - PRESERVED: Keep exact same map functionality
        self.grid_size_x = GRID_SIZE_X  # 11 columns
        self.grid_size_y = GRID_SIZE_Y  # 17 rows
        self.cell_size = 18  # PRESERVED: Same compact map size
        self.agv_marker = None
        self.target_marker = None
        self.anchor_markers = {}
        self.path_lines = []  # For path visualization
        self.grid_cells = {}
        
        # Performance optimization variables - PRESERVED
        self.gui_update_counter = 0
        self.last_agv_pos = (0, 0)
        self.position_update_threshold = 0.05  # GUI_POSITION_UPDATE_THRESHOLD

        # GUI setup
        self.setup_gui()
        self.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.bind_keys()
        
        self.start_controller()
        self.after(50, self.update_gui)  # Start updates

    def setup_gui(self):
        main_pane = ttk.PanedWindow(self, orient=tk.HORIZONTAL)
        main_pane.pack(fill="both", expand=True, padx=3, pady=3)  # FIXED: Reduced padding

        # Left pane
        left_frame = ttk.Frame(main_pane)
        main_pane.add(left_frame, weight=1)
        
        # =============================================================================
        # NAVIGATION FRAME - FIXED: Smaller buttons for 1920x1080
        # =============================================================================
        nav_frame = ttk.LabelFrame(left_frame, text="🎯 UWB Navigation Control")
        nav_frame.pack(fill="x", padx=3, pady=2)  # FIXED: Reduced padding
        
        nav_controls = ttk.Frame(nav_frame)
        nav_controls.pack(fill="x", padx=5, pady=3)  # FIXED: Reduced padding
        
        # Target input - FIXED: Smaller components
        ttk.Label(nav_controls, text="Target X:", font=("Arial", 9)).grid(row=0, column=0, padx=3, sticky="e")
        self.target_x_entry = ttk.Entry(nav_controls, width=8, font=("Arial", 9))  # FIXED: Smaller width
        self.target_x_entry.grid(row=0, column=1, padx=3)
        self.target_x_entry.insert(0, "2.0")
        
        ttk.Label(nav_controls, text="Target Y:", font=("Arial", 9)).grid(row=0, column=2, padx=3, sticky="e")
        self.target_y_entry = ttk.Entry(nav_controls, width=8, font=("Arial", 9))  # FIXED: Smaller width
        self.target_y_entry.grid(row=0, column=3, padx=3)
        self.target_y_entry.insert(0, "3.4")
        
        # FIXED: Navigation Button - Much smaller size
        self.navigate_btn = tk.Button(nav_controls, text="🎯 NAVIGATE", 
                                     command=self.navigate_to_target,
                                     bg="#27ae60", fg="white", font=("Arial", 10, "bold"),
                                     width=12, height=1, relief="raised", bd=2)  # FIXED: Smaller size
        self.navigate_btn.grid(row=0, column=4, padx=5)
        
        # FIXED: Quick navigation buttons - smaller
        quick_nav_frame = ttk.Frame(nav_controls)
        quick_nav_frame.grid(row=1, column=0, columnspan=5, pady=2)  # FIXED: Reduced padding
        
        quick_positions = [
            ("🏠 Center", 2.2, 3.4),
            ("🔴 A0", 3.8, 0.6),
            ("🟢 A1", 3.8, 6.2),
            ("🔵 A2", 0.2, 3.2)
        ]
        
        for i, (text, x, y) in enumerate(quick_positions):
            btn = tk.Button(quick_nav_frame, text=text, 
                           command=lambda x=x, y=y: self.quick_navigate(x, y),
                           bg="#3498db", fg="white", font=("Arial", 8), 
                           width=10, height=1)  # FIXED: Smaller buttons
            btn.pack(side="left", padx=1)  # FIXED: Reduced padding
        
        self.nav_status = tk.Label(nav_frame, text="Navigation: Ready", 
                                  font=("Arial", 9), fg="green")  # FIXED: Smaller font
        self.nav_status.pack(pady=2)  # FIXED: Reduced padding
        
        # =============================================================================
        # UWB STATUS FRAME - FIXED: Compact layout
        # =============================================================================
        uwb_frame = ttk.LabelFrame(left_frame, text="UWB Real-time Status")
        uwb_frame.pack(fill="x", padx=3, pady=2)  # FIXED: Reduced padding
        
        # Position display - FIXED: Compact
        pos_frame = ttk.Frame(uwb_frame)
        pos_frame.pack(fill="x", padx=3, pady=2)
        
        self.position_label = tk.Label(pos_frame, text="Position: X=0.00m, Y=0.00m", 
                                      font=("Arial", 10, "bold"), fg="blue")
        self.position_label.pack(side="left", padx=5)
        
        self.grid_label = tk.Label(pos_frame, text="Grid: (0, 0)", 
                                  font=("Arial", 10, "bold"), fg="green")
        self.grid_label.pack(side="left", padx=5)
        
        # Room info - FIXED: Smaller font
        room_info = tk.Label(pos_frame, 
                           text=f"Room: {ROOM_WIDTH}×{ROOM_HEIGHT}m ({GRID_SIZE_X}×{GRID_SIZE_Y})", 
                           font=("Arial", 8), fg="gray")
        room_info.pack(side="left", padx=5)
        
        # Kalman status - FIXED: Compact
        kalman_frame = ttk.Frame(uwb_frame)
        kalman_frame.pack(fill="x", padx=3, pady=1)
        
        self.kalman_status = tk.Label(kalman_frame, 
                                     text=f"Kalman: {'On' if UWB_KALMAN_ENABLED else 'Off'}", 
                                     font=("Arial", 9, "bold"), 
                                     fg="green" if UWB_KALMAN_ENABLED else "red")
        self.kalman_status.pack(side="left")
        
        if UWB_KALMAN_ENABLED:
            self.accuracy_label = tk.Label(kalman_frame, text="Accuracy: 0.000m", 
                                          font=("Arial", 9), fg="purple")
            self.accuracy_label.pack(side="left", padx=10)
        
        # Quality indicators - FIXED: Compact
        quality_frame = ttk.Frame(uwb_frame)
        quality_frame.pack(fill="x", padx=3, pady=1)
        
        self.updates_label = tk.Label(quality_frame, text="Updates: 0", 
                                     font=("Arial", 9))
        self.updates_label.pack(side="left")
        
        self.reset_uwb_btn = tk.Button(quality_frame, text="Reset UWB", 
                                      command=self.reset_uwb_diagnostics,
                                      bg="#e67e22", fg="white", font=("Arial", 8),
                                      width=8, height=1)  # FIXED: Smaller button
        self.reset_uwb_btn.pack(side="right", padx=3)
        
        # =============================================================================
        # ANCHOR DISTANCES - FIXED: More compact
        # =============================================================================
        anchor_frame = ttk.LabelFrame(uwb_frame, text="Anchor Distances")
        anchor_frame.pack(fill="x", padx=3, pady=2)
        
        self.anchor_labels = {}
        self.anchor_ground_labels = {}
        self.anchor_quality_labels = {}
        
        anchor_info = [
            ("A0", "red", (4.0, 0.4)),
            ("A1", "green", (4.0, 6.4)),
            ("A2", "blue", (0.0, 3.2))
        ]
        
        for i, (name, color, pos) in enumerate(anchor_info):
            frame = ttk.Frame(anchor_frame)
            frame.pack(side="left", padx=5)
            
            ttk.Label(frame, text=f"{name}:", font=("Arial", 8, "bold")).pack()
            
            distance_label = tk.Label(frame, text="3D: 0.00m", font=("Arial", 8), fg=color)
            distance_label.pack()
            self.anchor_labels[i] = distance_label
            
            ground_label = tk.Label(frame, text="2D: 0.00m", font=("Arial", 8), fg=color)
            ground_label.pack()
            self.anchor_ground_labels[i] = ground_label
            
            quality_label = tk.Label(frame, text="●", font=("Arial", 10), fg="gray")
            quality_label.pack()
            self.anchor_quality_labels[i] = quality_label
        
        # =============================================================================
        # MAP FRAME - PRESERVED: Keep exact same map functionality
        # =============================================================================
        map_frame = ttk.LabelFrame(left_frame, text=f"Position Map ({GRID_SIZE_X}×{GRID_SIZE_Y} Grid)")
        map_frame.pack(fill="x", padx=3, pady=2)  # FIXED: Keep map, reduce padding only
        
        # PRESERVED: Create grid map with exact same functionality
        self.create_grid_map(map_frame)

        # =============================================================================
        # RIGHT PANE: Controls and Sensor - FIXED: Compact layout
        # =============================================================================
        right_frame = ttk.Frame(main_pane)
        main_pane.add(right_frame, weight=1)

        # FIXED: Manual Control Frame - smaller
        control_frame = ttk.LabelFrame(right_frame, text="Manual Controls")
        control_frame.pack(fill="x", padx=3, pady=2)
        
        btn_frame = ttk.Frame(control_frame)
        btn_frame.pack(padx=5, pady=5)
        
        # FIXED: Smaller manual control buttons
        btn_style = {"width": 6, "height": 1, "bg": "#3498DB", "fg": "white", "font": ("Arial", 9)}
        tk.Button(btn_frame, text="↑", command=self.on_forward, **btn_style).grid(row=0, column=1, pady=1)
        tk.Button(btn_frame, text="←", command=self.on_left, **btn_style).grid(row=1, column=0, padx=1)
        tk.Button(btn_frame, text="STOP", command=self.on_stop, bg="#E74C3C", fg="white",
                  font=("Arial", 9), width=6, height=1).grid(row=1, column=1, pady=1)
        tk.Button(btn_frame, text="→", command=self.on_right, **btn_style).grid(row=1, column=2, padx=1)
        tk.Button(btn_frame, text="↓", command=self.on_backward, **btn_style).grid(row=2, column=1, pady=1)

        self.speed_var = tk.IntVar(value=int(MAX_SPEED))
        tk.Scale(control_frame, from_=0, to=30, orient=tk.HORIZONTAL, label="Speed (RPM)",
                 variable=self.speed_var, length=150, font=("Arial", 8)).pack(pady=2)  # FIXED: Smaller

        # FIXED: Task Frame - more compact
        task_frame = ttk.LabelFrame(right_frame, text="Tasks")
        task_frame.pack(fill="x", padx=3, pady=2)
        
        # Line Following Task - FIXED: Smaller buttons
        self.task1_btn = tk.Button(task_frame, text="Start Line Following", 
                                  command=self.start_line_following,
                                  bg="#1ABC9C", fg="white", font=("Arial", 9),
                                  width=18, height=1)  # FIXED: Smaller
        self.task1_btn.pack(pady=2)
        
        self.task1_status = tk.Label(task_frame, text="Line Following: Stopped", 
                                    font=("Arial", 9))
        self.task1_status.pack(pady=1)
        
        self.task1_error = tk.Label(task_frame, text="PID Error: 0.00", 
                                   font=("Arial", 9))
        self.task1_error.pack(pady=1)

        # Task 1 Navigation - FIXED: Smaller
        separator = ttk.Separator(task_frame, orient='horizontal')
        separator.pack(fill='x', pady=5)
        
        self.task1_nav_btn = tk.Button(task_frame, text="🎯 Task 1 (Grid 6,10)", 
                                      command=self.start_task1_navigation,
                                      bg="#9b59b6", fg="white", font=("Arial", 9),
                                      width=18, height=1)  # FIXED: Smaller
        self.task1_nav_btn.pack(pady=2)

        self.task1_nav_status = tk.Label(task_frame, text="Task 1: Ready", 
                                        font=("Arial", 9))
        self.task1_nav_status.pack(pady=1)

        # FIXED: PID settings - more compact
        pid_frame = ttk.Frame(task_frame)
        pid_frame.pack(fill="x", padx=3, pady=2)
        
        ttk.Label(pid_frame, text="Kp:", font=("Arial", 8)).grid(row=0, column=0, padx=1, sticky="e")
        self.kp_entry = ttk.Entry(pid_frame, width=6, font=("Arial", 8))  # FIXED: Smaller
        self.kp_entry.grid(row=0, column=1, padx=1)
        self.kp_entry.insert(0, str(INITIAL_PID_KP))
        
        ttk.Label(pid_frame, text="Ki:", font=("Arial", 8)).grid(row=0, column=2, padx=1, sticky="e")
        self.ki_entry = ttk.Entry(pid_frame, width=6, font=("Arial", 8))  # FIXED: Smaller
        self.ki_entry.grid(row=0, column=3, padx=1)
        self.ki_entry.insert(0, str(INITIAL_PID_KI))
        
        ttk.Label(pid_frame, text="Kd:", font=("Arial", 8)).grid(row=0, column=4, padx=1, sticky="e")
        self.kd_entry = ttk.Entry(pid_frame, width=6, font=("Arial", 8))  # FIXED: Smaller
        self.kd_entry.grid(row=0, column=5, padx=1)
        self.kd_entry.insert(0, str(INITIAL_PID_KD))
        
        tk.Button(pid_frame, text="Update", command=self.update_pid,
                  bg="#F1C40F", fg="black", font=("Arial", 8), width=6).grid(row=0, column=6, padx=3)

        # FIXED: Sensor Frame - more compact but preserve functionality
        sensor_frame = ttk.LabelFrame(right_frame, text="Magnetic Sensor Data")
        sensor_frame.pack(fill="both", expand=True, padx=3, pady=2)

        data_display = ttk.Frame(sensor_frame)
        data_display.pack(fill="x", padx=3, pady=2)
        
        ttk.Label(data_display, text="Median:", font=("Arial", 8)).grid(row=0, column=0, sticky="w")
        self.intermediate_value = ttk.Label(data_display, text="0", font=("Arial", 8, "bold"))
        self.intermediate_value.grid(row=0, column=1, padx=3, sticky="w")
        
        ttk.Label(data_display, text="Position:", font=("Arial", 8)).grid(row=1, column=0, sticky="w")
        self.position_value_label = ttk.Label(data_display, text="0x0000", font=("Arial", 8, "bold"))
        self.position_value_label.grid(row=1, column=1, padx=3, sticky="w")

        # PRESERVED: Sensor visualization - keep exact functionality
        ttk.Label(sensor_frame, text="Sensors: [1-8=RIGHT] | [9-16=LEFT]", font=("Arial", 8)).pack(pady=1)
        self.sensor_canvas = tk.Canvas(sensor_frame, width=400, height=50, bg="white")  # FIXED: Smaller height
        self.sensor_canvas.pack(fill="x", padx=3, pady=2)
        
        self.sensor_points = []
        for i in range(16):
            x = 20 + i * 23
            y = 25  # FIXED: Adjusted for smaller canvas
            circle = self.sensor_canvas.create_oval(x-6, y-6, x+6, y+6, fill="white", outline="black")
            text = self.sensor_canvas.create_text(x, y, text=str(i+1), font=("Arial", 6))
            self.sensor_points.append((circle, text))

        # FIXED: Status Frame - compact
        status_frame = ttk.Frame(right_frame)
        status_frame.pack(fill="x", padx=3, pady=2)
        
        self.status_var = tk.StringVar(value="Connecting...")
        self.status_label = tk.Label(status_frame, textvariable=self.status_var, 
                                    font=("Arial", 9), fg="red")
        self.status_label.pack()
        
        # FIXED: Motor feedback - compact
        feedback_frame = ttk.LabelFrame(right_frame, text="Motor Feedback")
        feedback_frame.pack(fill="x", padx=3, pady=2)
        
        self.lbl_motor1 = tk.Label(feedback_frame, text="Motor 1: Unknown", font=("Arial", 8))
        self.lbl_motor1.pack(anchor="w", padx=3)
        self.lbl_motor2 = tk.Label(feedback_frame, text="Motor 2: Unknown", font=("Arial", 8))
        self.lbl_motor2.pack(anchor="w", padx=3)

        # FIXED: Log text - smaller
        self.log_text = tk.Text(right_frame, height=4, width=40, font=("Arial", 8))  # FIXED: Smaller height
        self.log_text.pack(fill="x", padx=3, pady=2)
        self.log_handler = TextHandler(self.log_text)
        logger.addHandler(self.log_handler)

    # =============================================================================
    # MAP FUNCTIONS - PRESERVED: Keep exact same functionality
    # =============================================================================
    def create_grid_map(self, parent):
        """PRESERVED: Create exact same visual grid map functionality"""
        map_container = tk.Frame(parent, bg='white', relief=tk.SUNKEN, bd=2)
        map_container.pack(padx=3, pady=3)
        
        # PRESERVED: Calculate same canvas size
        canvas_width = self.grid_size_x * self.cell_size + 30
        canvas_height = self.grid_size_y * self.cell_size + 30
        
        self.map_canvas = tk.Canvas(map_container, 
                                   width=canvas_width,
                                   height=canvas_height,
                                   bg='white', highlightthickness=0)
        self.map_canvas.pack()
        
        # PRESERVED: Draw grid lines exactly same
        for i in range(self.grid_size_x + 1):
            x = i * self.cell_size + 15
            self.map_canvas.create_line(x, 15, x, self.grid_size_y * self.cell_size + 15,
                                       fill='#bdc3c7', width=1)
                                       
        for i in range(self.grid_size_y + 1):
            y = i * self.cell_size + 15
            self.map_canvas.create_line(15, y, self.grid_size_x * self.cell_size + 15, y,
                                       fill='#bdc3c7', width=1)
        
        # PRESERVED: Add coordinate labels exactly same
        for i in range(0, self.grid_size_x, 3):
            x = i * self.cell_size + self.cell_size/2 + 15
            self.map_canvas.create_text(x, 8, text=str(i), font=("Arial", 7))
                                       
        for i in range(0, self.grid_size_y, 3):
            y = i * self.cell_size + self.cell_size/2 + 15
            self.map_canvas.create_text(8, self.grid_size_y * self.cell_size - y + 15, 
                                       text=str(i), font=("Arial", 7))
        
        # PRESERVED: Create grid cells exactly same
        for x in range(self.grid_size_x):
            for y in range(self.grid_size_y):
                x1 = x * self.cell_size + 16
                y1 = (self.grid_size_y - 1 - y) * self.cell_size + 16
                x2 = x1 + self.cell_size - 2
                y2 = y1 + self.cell_size - 2
                
                rect = self.map_canvas.create_rectangle(x1, y1, x2, y2,
                                                       fill='', outline='',
                                                       width=0)
                self.grid_cells[(x, y)] = rect
        
        # PRESERVED: Create anchor markers exactly same
        anchor_positions = [
            (10, 1, "A0", "red"),
            (10, 16, "A1", "green"),
            (0, 8, "A2", "blue")
        ]
        
        for grid_x, grid_y, label, color in anchor_positions:
            canvas_x = grid_x * self.cell_size + self.cell_size/2 + 15
            canvas_y = (self.grid_size_y - 1 - grid_y) * self.cell_size + self.cell_size/2 + 15
            
            anchor = self.map_canvas.create_oval(canvas_x-6, canvas_y-6, canvas_x+6, canvas_y+6,
                                               fill=color, outline='black', width=2)
            anchor_text = self.map_canvas.create_text(canvas_x, canvas_y-12, text=label, 
                                                    font=("Arial", 8, "bold"), fill=color)
            
            self.anchor_markers[label] = (anchor, anchor_text)
        
        # PRESERVED: Highlight target grid exactly same
        target_grid_x, target_grid_y = TASK1_TARGET_GRID
        if 0 <= target_grid_x < self.grid_size_x and 0 <= target_grid_y < self.grid_size_y:
            canvas_x = target_grid_x * self.cell_size + self.cell_size/2 + 15
            canvas_y = (self.grid_size_y - 1 - target_grid_y) * self.cell_size + self.cell_size/2 + 15
            
            target_marker = self.map_canvas.create_rectangle(
                canvas_x - 8, canvas_y - 8, canvas_x + 8, canvas_y + 8,
                fill='#8e44ad', outline='#9b59b6', width=2
            )
            target_text = self.map_canvas.create_text(canvas_x, canvas_y + 15, 
                                                    text="Task1", font=("Arial", 8, "bold"), 
                                                    fill='#8e44ad')
        
        # PRESERVED: Create AGV marker exactly same
        self.agv_marker = self.map_canvas.create_oval(0, 0, 12, 12,
                                                     fill='#3498db', outline='#2980b9',
                                                     width=2)
        
        self.agv_direction = self.map_canvas.create_line(0, 0, 0, 0,
                                                        fill='#e74c3c', width=2,
                                                        arrow=tk.LAST)
        
        # PRESERVED: Initially hide AGV
        self.map_canvas.itemconfig(self.agv_marker, state='hidden')
        self.map_canvas.itemconfig(self.agv_direction, state='hidden')
        
        # PRESERVED: Mouse click event
        self.map_canvas.bind("<Button-1>", self.on_map_click)

    # PRESERVED: All coordinate conversion functions exactly same
    def real_to_canvas_coords(self, real_x, real_y):
        """PRESERVED: Convert real-world coordinates to canvas coordinates"""
        grid_x = real_x / CELL_SIZE
        grid_y = real_y / CELL_SIZE
        
        canvas_x = grid_x * self.cell_size + 15
        canvas_y = (self.grid_size_y - grid_y) * self.cell_size + 15
        
        return canvas_x, canvas_y

    def agv_real_to_canvas_coords(self, real_x, real_y):
        """PRESERVED: Convert AGV real-world coordinates to canvas coordinates"""
        grid_x = real_x / CELL_SIZE
        grid_y = real_y / CELL_SIZE
        
        canvas_x = grid_x * self.cell_size + 15
        canvas_y = grid_y * self.cell_size + 15
        
        return canvas_x, canvas_y

    def canvas_to_real_coords(self, canvas_x, canvas_y):
        """PRESERVED: Convert canvas coordinates to real-world coordinates"""
        grid_x = (canvas_x - 15) / self.cell_size
        grid_y = self.grid_size_y - (canvas_y - 15) / self.cell_size
        
        real_x = grid_x * CELL_SIZE
        real_y = grid_y * CELL_SIZE
        
        return real_x, real_y

    def update_agv_position(self, x, y, heading=90):
        """PRESERVED: Update AGV position with exact same functionality"""
        dx = x - self.last_agv_pos[0]
        dy = y - self.last_agv_pos[1]
        if (dx**2 + dy**2) < self.position_update_threshold**2:
            return
        
        self.last_agv_pos = (x, y)
        
        canvas_x, canvas_y = self.agv_real_to_canvas_coords(x, y)
        
        self.map_canvas.coords(self.agv_marker,
                              canvas_x - 6, canvas_y - 6,
                              canvas_x + 6, canvas_y + 6)
        
        if self.gui_update_counter % 3 == 0:
            dx = 10 * math.cos(math.radians(heading))
            dy = -10 * math.sin(math.radians(heading))
            self.map_canvas.coords(self.agv_direction,
                                  canvas_x, canvas_y,
                                  canvas_x + dx, canvas_y + dy)
        
        self.map_canvas.itemconfig(self.agv_marker, state='normal')
        self.map_canvas.itemconfig(self.agv_direction, state='normal')
        
        grid_x = int(x / CELL_SIZE)
        grid_y = int(y / CELL_SIZE)
        if 0 <= grid_x < self.grid_size_x and 0 <= grid_y < self.grid_size_y:
            for coord, rect in self.grid_cells.items():
                if coord == (grid_x, grid_y):
                    self.map_canvas.itemconfig(rect, fill='#e8f6f3', outline='#27ae60', width=2)
                else:
                    self.map_canvas.itemconfig(rect, fill='', outline='', width=0)

    def draw_planned_path(self, path_lines):
        """PRESERVED: Draw planned path exactly same"""
        for line in getattr(self, 'path_lines', []):
            self.map_canvas.delete(line)
        self.path_lines = []
        
        for from_pos, to_pos in path_lines:
            from_canvas = self.agv_real_to_canvas_coords(from_pos[0], from_pos[1])
            to_canvas = self.agv_real_to_canvas_coords(to_pos[0], to_pos[1])
            
            line = self.map_canvas.create_line(
                from_canvas[0], from_canvas[1],
                to_canvas[0], to_canvas[1],
                fill='#27ae60', width=3, tags='path'
            )
            self.path_lines.append(line)

    def update_target_position(self, x, y):
        """PRESERVED: Update target position exactly same"""
        if self.target_marker:
            self.map_canvas.delete(self.target_marker)
        
        canvas_x, canvas_y = self.real_to_canvas_coords(x, y)
        
        self.target_marker = self.map_canvas.create_rectangle(
            canvas_x - 8, canvas_y - 8,
            canvas_x + 8, canvas_y + 8,
            fill='#e74c3c', outline='#c0392b', width=2
        )

    def on_map_click(self, event):
        """PRESERVED: Handle click on map exactly same"""
        real_x, real_y = self.canvas_to_real_coords(event.x, event.y)
        
        real_x = max(0, min(ROOM_WIDTH, real_x))
        real_y = max(0, min(ROOM_HEIGHT, real_y))
        
        self.target_x_entry.delete(0, tk.END)
        self.target_x_entry.insert(0, f"{real_x:.1f}")
        self.target_y_entry.delete(0, tk.END)
        self.target_y_entry.insert(0, f"{real_y:.1f}")
        
        self.update_target_position(real_x, real_y)

    # =============================================================================
    # NAVIGATION FUNCTIONS - PRESERVED but connected to merged controller
    # =============================================================================
    def navigate_to_target(self):
        """Navigate to target position using merged controller"""
        if not self.controller.uwb_enabled:
            messagebox.showwarning("UWB Not Available", 
                                  "UWB navigation is not available. Check UWB connection.")
            return
            
        try:
            x = float(self.target_x_entry.get())
            y = float(self.target_y_entry.get())
            
            if not (0 <= x <= ROOM_WIDTH and 0 <= y <= ROOM_HEIGHT):
                raise ValueError(f"Coordinates must be within {ROOM_WIDTH}×{ROOM_HEIGHT}m")
            
            self.update_target_position(x, y)
            self.command_queue.put(("navigate_to", (x, y)))
            
            self.nav_status.config(text=f"🎯 Navigating to ({x:.1f}, {y:.1f})...", fg="blue")
            self.navigate_btn.config(text="🔄 Navigating...", bg="#e67e22")
            
            logger.info(f"Navigation started to ({x:.1f}, {y:.1f})")
            
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
                                  "UWB navigation required for Task 1.")
            return
        
        self.command_queue.put(("start_task1", None))
        logger.info("Task 1 navigation started")

    def reset_uwb_diagnostics(self):
        """Reset UWB diagnostics"""
        self.command_queue.put(("reset_uwb", None))
        logger.info("UWB diagnostics reset requested")

    # =============================================================================
    # UPDATE FUNCTIONS - PRESERVED functionality, optimized for merged controller
    # =============================================================================
    def update_anchor_distances(self):
        """Update anchor distance displays"""
        if hasattr(self.controller, 'uwb') and self.controller.uwb:
            distances_3d = self.controller.uwb.get_anchor_distances()
            distances_2d = self.controller.uwb.get_anchor_distances_2d()
            
            for i, label in self.anchor_labels.items():
                if i in distances_3d:
                    distance_3d = distances_3d[i]
                    distance_2d = distances_2d.get(i, 0)
                    
                    label.config(text=f"3D: {distance_3d:.3f}m")
                    self.anchor_ground_labels[i].config(text=f"2D: {distance_2d:.3f}m")
                    
                    if 0.1 <= distance_3d <= 25.0:
                        self.anchor_quality_labels[i].config(fg="green")
                    else:
                        self.anchor_quality_labels[i].config(fg="red")

    def update_diagnostics(self):
        """Update UWB diagnostics"""
        diagnostics = self.controller.get_uwb_diagnostics()
        
        if UWB_KALMAN_ENABLED and hasattr(self, 'accuracy_label'):
            avg_accuracy = diagnostics.get('average_accuracy', 0)
            self.accuracy_label.config(text=f"Accuracy: {avg_accuracy:.3f}m")
        
        update_count = diagnostics.get('update_count', 0)
        self.updates_label.config(text=f"Updates: {update_count}")

    def update_sensor_display(self):
        """PRESERVED: Update sensor display with exact functionality"""
        try:
            median_value, position_value = self.sensor_queue.get_nowait()
            if median_value is not None and position_value is not None:
                self.intermediate_value.config(text=f"{median_value:.1f}")
                self.position_value_label.config(text=f"0x{position_value:04X}")
                
                # PRESERVED: Update sensor visualization exactly same
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

    def update_status_display(self):
        """Update status display"""
        with self.controller.lock:
            self.status_var.set(f"State: {self.controller.state.name}")
            
            if self.controller.state == AGVState.LINE_FOLLOW:
                self.task1_status.config(text=f"Line Following: Active (Error: {self.controller.pid_error:.3f})")
                self.task1_btn.config(text="Stop Line Following", bg="#e74c3c")
            else:
                self.task1_status.config(text="Line Following: Stopped")
                self.task1_btn.config(text="Start Line Following", bg="#1ABC9C")
            
            self.task1_error.config(text=f"PID Error: {self.controller.pid_error:.3f}")
            
            # Update Task 1 Navigation status
            if self.controller.state == AGVState.TASK1_NAVIGATION:
                self.task1_nav_btn.config(text="🔄 Task 1 Running...", bg="#e67e22")
                if self.controller.navigation:
                    nav_status = self.controller.navigation.get_status()
                    progress = nav_status.get('progress', '0/0')
                    current_step = nav_status.get('current_step', 'None')
                    self.task1_nav_status.config(text=f"Task 1: {progress} - {current_step}")
                    
                    path_lines = nav_status.get('path_visualization', [])
                    if path_lines:
                        self.draw_planned_path(path_lines)
            else:
                self.task1_nav_btn.config(text="🎯 Task 1 (Grid 6,10)", bg="#9b59b6")
                if self.controller.state == AGVState.IDLE:
                    self.task1_nav_status.config(text="Task 1: Ready")
            
            # Update navigation button based on state
            if self.controller.state == AGVState.UWB_NAVIGATION:
                self.navigate_btn.config(text="🔄 Navigating...", bg="#e67e22")
                if self.controller.navigation:
                    nav_status = self.controller.navigation.get_status()
                    distance = nav_status.get('distance_to_target')
                    if distance is not None:
                        self.nav_status.config(text=f"🎯 Navigation: {nav_status['state']} (Dist: {distance:.3f}m)", fg="blue")
                    else:
                        self.nav_status.config(text=f"🎯 Navigation: {nav_status['state']}", fg="blue")
            else:
                self.navigate_btn.config(text="🎯 NAVIGATE", bg="#27ae60")
                if self.controller.state == AGVState.IDLE:
                    self.nav_status.config(text="Navigation: Ready", fg="green")
                elif self.controller.state == AGVState.LINE_FOLLOW:
                    self.nav_status.config(text="Navigation: Line following mode", fg="purple")
            
            rpm1, cur1 = self.controller.motor_control.prev_fb_rpm[0], self.controller.motor_control.prev_fb_cur[0]
            rpm2, cur2 = self.controller.motor_control.prev_fb_rpm[1], self.controller.motor_control.prev_fb_cur[1]
            self.lbl_motor1.config(text=f"Motor 1: RPM {rpm1}, Current {cur1:.2f}A")
            self.lbl_motor2.config(text=f"Motor 2: RPM {rpm2}, Current {cur2:.2f}A")

    def update_gui(self):
        """Main GUI update with preserved functionality"""
        self.gui_update_counter += 1
        
        # UWB updates
        if self.controller.uwb_enabled and self.controller.uwb:
            x, y, z = self.controller.get_uwb_position()
            
            self.position_label.config(text=f"Position: X={x:.3f}m, Y={y:.3f}m")
            
            if hasattr(self.controller.uwb, 'adj_grid'):
                grid_x, grid_y = self.controller.uwb.adj_grid(x, y)
                if grid_x is not None and grid_y is not None:
                    self.grid_label.config(text=f"Grid: ({grid_x}, {grid_y})")
            
            # PRESERVED: Update map position with exact same functionality
            if x > 0 or y > 0:
                self.update_agv_position(x, y)
        
        # Heavy operations less frequently
        if self.gui_update_counter % 3 == 0:
            self.update_anchor_distances()
            self.update_diagnostics()
        
        # PRESERVED: Sensor data and status updates
        self.update_sensor_display()
        self.update_status_display()
        
        self.after(50, self.update_gui)

    # =============================================================================
    # CONTROL FUNCTIONS - PRESERVED and connected to merged controller
    # =============================================================================
    def bind_keys(self):
        self.bind("<Up>", lambda e: self.on_forward())
        self.bind("<Down>", lambda e: self.on_backward())
        self.bind("<Left>", lambda e: self.on_left())
        self.bind("<Right>", lambda e: self.on_right())
        self.bind("<space>", lambda e: self.on_stop())
        self.bind("<Return>", lambda e: self.navigate_to_target())
        self.bind("<Control-g>", lambda e: self.navigate_to_target())

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

    def start_line_following(self):
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