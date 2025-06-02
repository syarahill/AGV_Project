import threading
import queue
import time
import logging
from collections import deque
from config import *
from sensor_reader import SensorReader
from motor_control import MotorControl

# Try to import UWB modules, but continue if they don't exist yet
try:
    from uwb_reader import UWBReader
    from navigation import PathNavigation, NavigationState
except ImportError as e:
    logging.warning("UWB modules not found - continuing without UWB support")
    UWBReader = None
    PathNavigation = None
    NavigationState = None

logger = logging.getLogger("Controller")

class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = time.time()
        self.max_integral = 100.0  # Prevent integral windup

    def compute(self, error, dt):
        if dt <= 0:
            dt = 0.001  # Prevent division by zero
            
        # Integral with windup protection
        self.integral += error * dt
        if self.integral > self.max_integral:
            self.integral = self.max_integral
        elif self.integral < -self.max_integral:
            self.integral = -self.max_integral
            
        # Derivative
        derivative = (error - self.last_error) / dt
        
        # PID output
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.last_error = error
        self.last_time = time.time()
        
        return output

    def reset(self):
        """Reset PID controller"""
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = time.time()

class Controller:
    def __init__(self, enable_uwb=True):
        self.sensor = SensorReader()
        self.motor_control = MotorControl()
        
        # OPTIMIZED: Initialize UWB with better error handling
        self.uwb = None
        self.navigation = None
        self.uwb_enabled = enable_uwb
        
        if enable_uwb and UWBReader and PathNavigation:
            try:
                self.uwb = UWBReader(UWB_PORT)
                if self.uwb.connect():
                    self.navigation = PathNavigation(self.uwb, self.motor_control)
                    logger.info("Path navigation initialized")
                else:
                    logger.warning("Failed to initialize UWB navigation - continuing without UWB")
                    self.uwb_enabled = False
            except Exception as e:
                logger.warning(f"UWB initialization error: {e} - continuing without UWB")
                self.uwb_enabled = False
        else:
            self.uwb_enabled = False
        
        self.command_queue = queue.Queue()
        self.sensor_queue = queue.Queue()
        self.state = AGVState.DISCONNECTED
        self.pid = PID(INITIAL_PID_KP, INITIAL_PID_KI, INITIAL_PID_KD)
        self.desired_rpm = [0, 0]
        self.pid_error = 0.0
        self.lock = threading.Lock()
        self.running = False
        
        # OPTIMIZED: Line following improvements
        self.line_position_history = deque(maxlen=MOVING_AVERAGE_WINDOW)
        self.line_lost_timer = 0
        self.last_known_position = IDEAL_CENTER
        self.line_detected = False
        
        # OPTIMIZED: UWB position tracking with performance monitoring
        self.current_uwb_position = (0.0, 0.0, 0.0)
        self.uwb_position_quality = {}
        self.uwb_last_update = 0
        
        # OPTIMIZED: UWB diagnostics with rate limiting
        self.uwb_update_count = 0
        self.uwb_error_count = 0
        self.position_accuracy_log = deque(maxlen=50)  # Reduced from 100 for performance

    def start(self):
        self.running = True
        threading.Thread(target=self._sensor_loop, daemon=True).start()
        threading.Thread(target=self._control_loop, daemon=True).start()
        if self.uwb_enabled and self.uwb:
            threading.Thread(target=self._uwb_loop, daemon=True).start()
            threading.Thread(target=self._uwb_diagnostics_loop, daemon=True).start()

    def stop(self):
        self.running = False
        self.sensor.close()
        self.motor_control.close()
        if self.uwb:
            self.uwb.close()

    def reset_system_state(self):
        """OPTIMIZED: Reset system to safe state on errors"""
        self.motor_control.emergency_stop()
        self.state = AGVState.IDLE
        if self.navigation:
            self.navigation.stop()
        if self.uwb:
            self.uwb.reset_position_filter()
        logger.info("System state reset to safe mode")

    def _sensor_loop(self):
        if not self.sensor.connect():
            self.state = AGVState.ERROR
            return
        while self.running:
            try:
                data = self.sensor.read_data()
                if data:
                    self.sensor_queue.put(data)
                time.sleep(SENSOR_READ_RATE / 1000.0)
            except Exception as e:
                logger.error(f"Sensor loop error: {e}")
                time.sleep(0.1)

    def _uwb_loop(self):
        """OPTIMIZED: Fast UWB position monitoring"""
        if not self.uwb:
            return
            
        self.uwb.continuous_read()  # Start the UWB continuous reading
        
        # Monitor the position updates with optimized rate
        while self.running and self.uwb:
            try:
                with self.lock:
                    new_position = self.uwb.get_current_position()
                    
                    # Check if position actually updated
                    if new_position != self.current_uwb_position:
                        self.current_uwb_position = new_position
                        self.uwb_last_update = time.time()
                        self.uwb_update_count += 1
                        
                        # OPTIMIZED: Get simplified position quality metrics
                        self.uwb_position_quality = self.uwb.get_position_quality()
                        
                        # OPTIMIZED: Simplified accuracy logging
                        if self.uwb_position_quality and UWB_KALMAN_ENABLED:
                            raw_pos = self.uwb_position_quality.get('raw_position')
                            filtered_pos = self.uwb_position_quality.get('filtered_position')
                            if raw_pos and filtered_pos:
                                accuracy = ((raw_pos[0] - filtered_pos[0])**2 + (raw_pos[1] - filtered_pos[1])**2)**0.5
                                self.position_accuracy_log.append(accuracy)
                
                time.sleep(0.03)  # OPTIMIZED: 33Hz update rate for monitoring
                
            except Exception as e:
                logger.error(f"UWB loop error: {e}")
                self.uwb_error_count += 1
                # OPTIMIZED: Reset system state on critical errors
                if self.uwb_error_count > 10:
                    self.reset_system_state()
                    self.uwb_error_count = 0
                time.sleep(0.1)

    def _uwb_diagnostics_loop(self):
        """OPTIMIZED: Faster UWB diagnostics with simplified monitoring"""
        if not self.uwb:
            return
            
        last_diagnostic_time = time.time()
        
        while self.running and self.uwb:
            current_time = time.time()
            
            # OPTIMIZED: Run diagnostics every 5 seconds instead of 10
            if current_time - last_diagnostic_time >= 5:
                try:
                    with self.lock:
                        quality = self.uwb_position_quality
                        
                        # OPTIMIZED: Calculate average position accuracy if we have data
                        avg_accuracy = 0
                        if self.position_accuracy_log:
                            avg_accuracy = sum(self.position_accuracy_log) / len(self.position_accuracy_log)
                        
                        # Check for position timeout
                        time_since_update = current_time - self.uwb_last_update
                        
                        # OPTIMIZED: Simplified diagnostic logging
                        logger.debug(f"UWB Status - Updates: {self.uwb_update_count}, "
                                   f"Errors: {self.uwb_error_count}, "
                                   f"Last update: {time_since_update:.1f}s ago, "
                                   f"Avg accuracy: {avg_accuracy:.3f}m")
                        
                        # OPTIMIZED: Simplified error reset with lower thresholds
                        if (quality.get('position_jump_count', 0) > 10 or 
                            quality.get('trilateration_error_count', 0) > 50):
                            logger.warning("High error count detected, resetting UWB position filter")
                            self.uwb.reset_position_filter()
                        
                        # Warning if no recent updates
                        if time_since_update > 3:
                            logger.warning(f"No UWB position updates for {time_since_update:.1f} seconds")
                    
                    last_diagnostic_time = current_time
                    
                except Exception as e:
                    logger.error(f"UWB diagnostics error: {e}")
            
            time.sleep(2)  # OPTIMIZED: Check every 2 seconds instead of 1

    def navigate_to_position(self, x: float, y: float):
        """ENHANCED: Navigate to a specific position using UWB"""
        if not self.navigation:
            logger.error("UWB navigation not available")
            return False
            
        # OPTIMIZED: Validate coordinates quickly
        if not (0 <= x <= ROOM_WIDTH and 0 <= y <= ROOM_HEIGHT):
            logger.error(f"Target ({x}, {y}) outside room bounds")
            return False
            
        # Convert to grid coordinates
        grid_x = int(x / CELL_SIZE)
        grid_y = int(y / CELL_SIZE)
        
        if self.navigation.set_target_grid(grid_x, grid_y):
            self.state = AGVState.UWB_NAVIGATION
            logger.info(f"Starting path navigation to grid ({grid_x}, {grid_y})")
            return True
        return False

    def start_task1(self):
        """NEW: Start Task 1 - Move to grid position (6,10)"""
        if not self.navigation:
            logger.error("Navigation not available for Task 1")
            return False
        
        target_x, target_y = TASK1_TARGET_GRID
        
        if self.navigation.set_target_grid(target_x, target_y):
            self.state = AGVState.TASK1_NAVIGATION
            logger.info(f"🎯 Task 1 started: Moving to grid ({target_x}, {target_y})")
            return True
        return False

    def get_uwb_position(self) -> tuple:
        """Get current UWB position"""
        with self.lock:
            return self.current_uwb_position

    def get_uwb_diagnostics(self) -> dict:
        """OPTIMIZED: Get simplified UWB positioning diagnostics"""
        with self.lock:
            avg_accuracy = 0
            if self.position_accuracy_log:
                avg_accuracy = sum(self.position_accuracy_log) / len(self.position_accuracy_log)
            
            return {
                'update_count': self.uwb_update_count,
                'error_count': self.uwb_error_count,
                'time_since_update': time.time() - self.uwb_last_update,
                'average_accuracy': avg_accuracy,
                'position_quality': self.uwb_position_quality.copy() if self.uwb_position_quality else {},
                'kalman_enabled': UWB_KALMAN_ENABLED
            }

    def reset_uwb_diagnostics(self):
        """Reset UWB diagnostic counters"""
        with self.lock:
            self.uwb_update_count = 0
            self.uwb_error_count = 0
            self.position_accuracy_log.clear()
        
        if self.uwb:
            self.uwb.reset_position_filter()
        
        logger.info("UWB diagnostics reset")

    def _calculate_line_position(self, position_value):
        """OPTIMIZED: Calculate line position with faster logic"""
        # Get active sensors (inverted logic - bit 0 means sensor detects line)
        active_sensors = [i + 1 for i in range(16) if not (position_value & (1 << i))]
        
        if not active_sensors:
            return None, False, "No line detected"
            
        # OPTIMIZED: Simplified noise filtering
        if len(active_sensors) > MAX_SENSORS_FOR_LINE:
            return None, False, f"Too many sensors: {active_sensors}"
            
        # OPTIMIZED: Fast position calculation
        if len(active_sensors) >= MIN_SENSORS_FOR_LINE:
            line_position = sum(active_sensors) / len(active_sensors)
            
            # OPTIMIZED: Simplified moving average
            self.line_position_history.append(line_position)
            smoothed_position = sum(self.line_position_history) / len(self.line_position_history)
            
            return smoothed_position, True, f"Sensors: {active_sensors}"
            
        return None, False, f"Not enough sensors: {active_sensors}"

    def _calculate_correction(self, line_position):
        """OPTIMIZED: Calculate steering correction with faster PID"""
        error = line_position - IDEAL_CENTER
        
        # Get PID correction
        current_time = time.time()
        dt = current_time - self.pid.last_time
        correction = self.pid.compute(error, dt)
        
        # OPTIMIZED: Simplified correction limiting
        correction = max(-MAX_CORRECTION, min(MAX_CORRECTION, correction))
        if abs(correction) < MIN_CORRECTION:
            correction = 0  # Dead zone
            
        # Apply correction logic
        left_speed = MAX_SPEED - correction
        right_speed = -(MAX_SPEED + correction)
        
        return [left_speed, right_speed], error, correction

    def _control_loop(self):
        last_motor_update = 0
        while self.running:
            current_time = time.time()
            
            # OPTIMIZED: Process commands without blocking
            try:
                while not self.command_queue.empty():
                    cmd = self.command_queue.get_nowait()
                    if cmd[0] == "set_state":
                        new_state = cmd[1]
                        if new_state != self.state:
                            logger.info(f"State change: {self.state.name} -> {new_state.name}")
                            if new_state == AGVState.LINE_FOLLOW:
                                self.pid.reset()
                            self.state = new_state
                    elif cmd[0] == "set_speed":
                        self.desired_rpm = cmd[1]
                    elif cmd[0] == "update_pid":
                        self.pid.kp, self.pid.ki, self.pid.kd = cmd[1]
                        logger.info(f"PID updated: Kp={self.pid.kp}, Ki={self.pid.ki}, Kd={self.pid.kd}")
                    elif cmd[0] == "navigate_to":
                        x, y = cmd[1]
                        self.navigate_to_position(x, y)
                    elif cmd[0] == "start_task1":
                        self.start_task1()
                    elif cmd[0] == "reset_uwb":
                        self.reset_uwb_diagnostics()
            except queue.Empty:
                pass
            except Exception as e:
                logger.error(f"Command processing error: {e}")
                self.reset_system_state()

            # OPTIMIZED: Motor update rate control
            if current_time - last_motor_update < MOTOR_UPDATE_RATE / 1000.0:
                time.sleep(0.001)
                continue
                
            last_motor_update = current_time

            try:
                # ENHANCED: Path Navigation logic for UWB and Task 1
                if self.state in [AGVState.UWB_NAVIGATION, AGVState.TASK1_NAVIGATION] and self.navigation:
                    nav_state = self.navigation.update()
                    
                    if nav_state == NavigationState.REACHED_TARGET:
                        self.state = AGVState.IDLE
                        self.desired_rpm = [0, 0]
                        if self.state == AGVState.TASK1_NAVIGATION:
                            logger.info("🎯 Task 1 completed successfully!")
                        else:
                            logger.info("🎯 Navigation target reached!")

                # OPTIMIZED: Line following logic
                elif self.state == AGVState.LINE_FOLLOW:
                    try:
                        # Get latest sensor data without blocking
                        median_value = None
                        position_value = None
                        while not self.sensor_queue.empty():
                            median_value, position_value = self.sensor_queue.get_nowait()
                        
                        if median_value is not None and position_value is not None:
                            # Calculate line position
                            line_position, line_found, debug_info = self._calculate_line_position(position_value)
                            
                            if line_found:
                                # Line detected - calculate correction
                                speeds, error, correction = self._calculate_correction(line_position)
                                self.desired_rpm = speeds
                                self.pid_error = error
                                self.last_known_position = line_position
                                self.line_lost_timer = 0
                                self.line_detected = True
                                
                            else:
                                # Line lost - handle gracefully
                                if self.line_lost_timer == 0:
                                    self.line_lost_timer = current_time
                                
                                if current_time - self.line_lost_timer > LINE_LOST_TIMEOUT:
                                    self.desired_rpm = [0, 0]
                                    self.state = AGVState.LINE_LOST
                                    logger.warning(f"Line lost! {debug_info}")
                                
                    except queue.Empty:
                        pass
                
                elif self.state == AGVState.LINE_LOST:
                    # OPTIMIZED: Simple search pattern
                    self.desired_rpm = [5, 5]  # Slow turn right to search
                
                # OPTIMIZED: Send motor commands with error handling
                with self.lock:
                    if self.motor_control.ser and self.motor_control.ser.is_open:
                        self.motor_control.send_rpm(MOTOR_ID_LEFT, self.desired_rpm[0])
                        self.motor_control.send_rpm(MOTOR_ID_RIGHT, self.desired_rpm[1])

            except Exception as e:
                logger.error(f"Critical error in control loop: {e}")
                self.reset_system_state()
                time.sleep(1)  # Prevent rapid error loops