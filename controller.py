# =============================================================================
# UNIFIED CONTROLLER - MERGED FROM OLD AND NEW CODE
# This controller combines the best of both systems:
# - Old code: Reliable line following, motor control, sensor reading, PID
# - New code: UWB positioning, grid navigation, non-blocking threading
# =============================================================================

import threading
import queue
import time
import logging
from collections import deque
from config import *
from sensor_reader import SensorReader  # FROM OLD CODE - reliable sensor communication
from motor_control import MotorControl   # FROM OLD CODE - reliable motor communication

# Try to import UWB modules, but continue if they don't exist yet
try:
    from uwb_reader import UWBReader      # FROM NEW CODE - proven UWB positioning
    from navigation import PathNavigation, NavigationState  # FROM NEW CODE - grid navigation
except ImportError as e:
    logging.warning("UWB modules not found - continuing without UWB support")
    UWBReader = None
    PathNavigation = None
    NavigationState = None

logger = logging.getLogger("Controller")

class PID:
    """PID Controller - FROM OLD CODE (EXACT PRESERVATION) with required values P=2.0, I=0.0, D=0.5"""
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = time.time()  # FROM OLD CODE - proper initialization
        self.max_integral = 100.0  # Prevent integral windup

    def compute(self, error, dt):
        """FROM OLD CODE (EXACT PRESERVATION) - proven PID calculation"""
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
        """Reset PID controller - FROM OLD CODE (EXACT PRESERVATION)"""
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = time.time()

class Controller:
    def __init__(self, enable_uwb=True):
        """Unified controller combining old and new code strengths"""
        
        # =============================================================================
        # CORE COMPONENTS - FROM OLD CODE (reliable communication)
        # =============================================================================
        self.sensor = SensorReader()    # FROM OLD CODE - reliable sensor communication
        self.motor_control = MotorControl()  # FROM OLD CODE - reliable motor communication
        
        # =============================================================================
        # UWB COMPONENTS - FROM NEW CODE (proven positioning)
        # =============================================================================
        self.uwb = None
        self.navigation = None
        self.uwb_enabled = enable_uwb
        
        if enable_uwb and UWBReader and PathNavigation:
            try:
                self.uwb = UWBReader(UWB_PORT)  # FROM NEW CODE - proven UWB system
                if self.uwb.connect():
                    self.navigation = PathNavigation(self.uwb, self.motor_control)  # FROM NEW CODE
                    logger.info("UWB positioning and path navigation initialized")
                else:
                    logger.warning("Failed to initialize UWB navigation - continuing without UWB")
                    self.uwb_enabled = False
            except Exception as e:
                logger.warning(f"UWB initialization error: {e} - continuing without UWB")
                self.uwb_enabled = False
        else:
            self.uwb_enabled = False
        
        # =============================================================================
        # THREADING AND QUEUES - FROM OLD CODE (reliable, proven threading)
        # =============================================================================
        self.command_queue = queue.Queue()
        self.sensor_queue = queue.Queue()
        self.state = AGVState.DISCONNECTED
        
        # =============================================================================
        # PID CONTROLLER - FROM OLD CODE (EXACT VALUES REQUIRED)
        # =============================================================================
        self.pid = PID(INITIAL_PID_KP, INITIAL_PID_KI, INITIAL_PID_KD)  # P=2.0, I=0.0, D=0.5
        
        # =============================================================================
        # MOTOR AND STATE MANAGEMENT - FROM OLD CODE
        # =============================================================================
        self.desired_rpm = [0, 0]
        self.pid_error = 0.0
        self.lock = threading.Lock()
        self.running = False
        
        # =============================================================================
        # LINE FOLLOWING - FROM OLD CODE (EXACT PRESERVATION)
        # =============================================================================
        self.line_position_history = deque(maxlen=MOVING_AVERAGE_WINDOW)  # FROM OLD CODE
        self.line_lost_timer = 0
        self.last_known_position = IDEAL_CENTER
        self.line_detected = False
        
        # =============================================================================
        # UWB POSITION TRACKING - FROM NEW CODE
        # =============================================================================
        self.current_uwb_position = (0.0, 0.0, 0.0)
        self.uwb_position_quality = {}
        self.uwb_last_update = 0
        
        # UWB diagnostics
        self.uwb_update_count = 0
        self.uwb_error_count = 0
        self.position_accuracy_log = deque(maxlen=50)

    def start(self):
        """Start all system threads - USING OLD CODE'S RELIABLE THREADING"""
        self.running = True
        
        # FROM OLD CODE - reliable sensor and control threads
        threading.Thread(target=self._sensor_loop, daemon=True).start()
        threading.Thread(target=self._control_loop, daemon=True).start()
        
        # FROM NEW CODE - UWB threads (only if UWB enabled)
        if self.uwb_enabled and self.uwb:
            threading.Thread(target=self._uwb_loop, daemon=True).start()
            threading.Thread(target=self._uwb_diagnostics_loop, daemon=True).start()

    def stop(self):
        """Stop all system components"""
        self.running = False
        self.sensor.close()      # FROM OLD CODE
        self.motor_control.close()  # FROM OLD CODE
        if self.uwb:
            self.uwb.close()     # FROM NEW CODE

    def reset_system_state(self):
        """Reset system to safe state on errors - FROM NEW CODE"""
        self.motor_control.emergency_stop()
        self.state = AGVState.IDLE
        if self.navigation:
            self.navigation.stop()
        if self.uwb:
            self.uwb.reset_position_filter()
        logger.info("System state reset to safe mode")

    # =============================================================================
    # SENSOR LOOP - FROM OLD CODE (EXACT PRESERVATION)
    # =============================================================================
    def _sensor_loop(self):
        """Sensor reading loop - FROM OLD CODE (proven reliable)"""
        if not self.sensor.connect():
            self.state = AGVState.ERROR
            return
        while self.running:
            try:
                data = self.sensor.read_data()
                if data:
                    self.sensor_queue.put(data)
                time.sleep(SENSOR_READ_RATE / 1000.0)  # FROM OLD CODE - proven timing
            except Exception as e:
                logger.error(f"Sensor loop error: {e}")
                time.sleep(0.1)

    # =============================================================================
    # UWB LOOP - FROM NEW CODE (EXACT PRESERVATION)
    # =============================================================================
    def _uwb_loop(self):
        """UWB position monitoring - FROM NEW CODE (proven positioning)"""
        if not self.uwb:
            return
            
        self.uwb.continuous_read()  # Start the UWB continuous reading
        
        while self.running and self.uwb:
            try:
                with self.lock:
                    new_position = self.uwb.get_current_position()
                    
                    if new_position != self.current_uwb_position:
                        self.current_uwb_position = new_position
                        self.uwb_last_update = time.time()
                        self.uwb_update_count += 1
                        
                        self.uwb_position_quality = self.uwb.get_position_quality()
                        
                        if self.uwb_position_quality and UWB_KALMAN_ENABLED:
                            raw_pos = self.uwb_position_quality.get('raw_position')
                            filtered_pos = self.uwb_position_quality.get('filtered_position')
                            if raw_pos and filtered_pos:
                                accuracy = ((raw_pos[0] - filtered_pos[0])**2 + (raw_pos[1] - filtered_pos[1])**2)**0.5
                                self.position_accuracy_log.append(accuracy)
                
                time.sleep(0.03)  # FROM NEW CODE - proven UWB timing
                
            except Exception as e:
                logger.error(f"UWB loop error: {e}")
                self.uwb_error_count += 1
                if self.uwb_error_count > 10:
                    self.reset_system_state()
                    self.uwb_error_count = 0
                time.sleep(0.1)

    def _uwb_diagnostics_loop(self):
        """UWB diagnostics - FROM NEW CODE (exact preservation)"""
        if not self.uwb:
            return
            
        last_diagnostic_time = time.time()
        
        while self.running and self.uwb:
            current_time = time.time()
            
            if current_time - last_diagnostic_time >= 5:
                try:
                    with self.lock:
                        quality = self.uwb_position_quality
                        
                        avg_accuracy = 0
                        if self.position_accuracy_log:
                            avg_accuracy = sum(self.position_accuracy_log) / len(self.position_accuracy_log)
                        
                        time_since_update = current_time - self.uwb_last_update
                        
                        logger.debug(f"UWB Status - Updates: {self.uwb_update_count}, "
                                   f"Errors: {self.uwb_error_count}, "
                                   f"Last update: {time_since_update:.1f}s ago, "
                                   f"Avg accuracy: {avg_accuracy:.3f}m")
                        
                        if (quality.get('position_jump_count', 0) > 10 or 
                            quality.get('trilateration_error_count', 0) > 50):
                            logger.warning("High error count detected, resetting UWB position filter")
                            self.uwb.reset_position_filter()
                        
                        if time_since_update > 3:
                            logger.warning(f"No UWB position updates for {time_since_update:.1f} seconds")
                    
                    last_diagnostic_time = current_time
                    
                except Exception as e:
                    logger.error(f"UWB diagnostics error: {e}")
            
            time.sleep(2)

    # =============================================================================
    # LINE FOLLOWING CALCULATIONS - FROM OLD CODE (EXACT PRESERVATION)
    # =============================================================================
    def _calculate_line_position(self, position_value):
        """Calculate line position - FROM OLD CODE (EXACT PRESERVATION)"""
        # Get active sensors (inverted logic - bit 0 means sensor detects line)
        active_sensors = [i + 1 for i in range(16) if not (position_value & (1 << i))]
        
        if not active_sensors:
            return None, False, "No line detected"
                
        # Filter out noise - too many sensors might indicate error
        if len(active_sensors) > MAX_SENSORS_FOR_LINE:
            logger.warning(f"Too many sensors active ({len(active_sensors)}), possible noise")
            return None, False, f"Too many sensors: {active_sensors}"
                
        # Calculate weighted position
        if len(active_sensors) >= MIN_SENSORS_FOR_LINE:
            # Simple average
            line_position = sum(active_sensors) / len(active_sensors)
                    
            # Add to moving average
            self.line_position_history.append(line_position)
                    
            # Use moving average for smoother control
            smoothed_position = sum(self.line_position_history) / len(self.line_position_history)
                    
            # Determine which side the line is on for debugging
            right_side_active = [s for s in active_sensors if s in LOW_SENSORS]  # Right side (1-8)
            left_side_active = [s for s in active_sensors if s in HIGH_SENSORS]  # Left side (9-16)
                    
            side_info = f"Right:{right_side_active}, Left:{left_side_active}"
            logger.debug(f"Active sensors: {active_sensors}, Position: {smoothed_position:.2f}, {side_info}")
                    
            return smoothed_position, True, side_info
                
        return None, False, f"Not enough sensors: {active_sensors}"

    def _calculate_correction(self, line_position):
        """Calculate steering correction - FROM OLD CODE (EXACT PRESERVATION)"""
        # Error calculation: positive error means line is on LEFT side (sensors 9-16)
        # negative error means line is on RIGHT side (sensors 1-8)
        error = line_position - IDEAL_CENTER
        
        # Get PID correction
        current_time = time.time()
        dt = current_time - self.pid.last_time
        correction = self.pid.compute(error, dt)
        
        # Limit correction
        if correction > MAX_CORRECTION:
            correction = MAX_CORRECTION
        elif correction < -MAX_CORRECTION:
            correction = -MAX_CORRECTION
        elif abs(correction) < MIN_CORRECTION:
            correction = 0  # Dead zone to prevent oscillation
                
        # Apply correction logic:
        # If error > 0: line is on LEFT side (high sensors), AGV too far RIGHT, turn LEFT
        # If error < 0: line is on RIGHT side (low sensors), AGV too far LEFT, turn RIGHT
        
        # Motor speeds: [left_motor, right_motor]
        # Turn LEFT: slow down left motor, speed up right motor
        # Turn RIGHT: speed up left motor, slow down right motor
        left_speed = MAX_SPEED - correction   # Reduce left speed to turn left
        right_speed = -(MAX_SPEED + correction)  # Increase right speed to turn left (negative because of motor direction)
        
        return [left_speed, right_speed], error, correction

    # =============================================================================
    # UWB NAVIGATION METHODS - FROM NEW CODE (EXACT PRESERVATION)
    # =============================================================================
    def navigate_to_position(self, x: float, y: float):
        """Navigate to a specific position using UWB - FROM NEW CODE"""
        if not self.navigation:
            logger.error("UWB navigation not available")
            return False
            
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
        """Start Task 1 - Move to grid position (6,10) - FROM NEW CODE"""
        if not self.navigation:
            logger.error("Navigation not available for Task 1")
            return False
        
        target_x, target_y = TASK1_TARGET_GRID
        
        if self.navigation.set_target_grid(target_x, target_y):
            self.state = AGVState.TASK1_NAVIGATION
            logger.info(f"🎯 Task 1 started: Moving to grid ({target_x}, {target_y})")
            return True
        return False

    # =============================================================================
    # UNIFIED CONTROL LOOP - MERGED FROM BOTH CODES
    # =============================================================================
    def _control_loop(self):
        """Main control loop - MERGED from old and new code"""
        last_motor_update = 0
        
        while self.running:
            current_time = time.time()
            
            # Process commands - FROM OLD CODE structure, enhanced with new commands
            try:
                while not self.command_queue.empty():
                    cmd = self.command_queue.get_nowait()
                    if cmd[0] == "set_state":
                        new_state = cmd[1]
                        if new_state != self.state:
                            logger.info(f"State change: {self.state.name} -> {new_state.name}")
                            if new_state == AGVState.LINE_FOLLOW:
                                self.pid.reset()  # FROM OLD CODE - reset PID when starting line follow
                            self.state = new_state
                    elif cmd[0] == "set_speed":
                        self.desired_rpm = cmd[1]
                    elif cmd[0] == "update_pid":
                        self.pid.kp, self.pid.ki, self.pid.kd = cmd[1]
                        logger.info(f"PID updated: Kp={self.pid.kp}, Ki={self.pid.ki}, Kd={self.pid.kd}")
                    # NEW COMMANDS from new code for UWB navigation
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

            # Motor update rate control - FROM OLD CODE (proven timing)
            if current_time - last_motor_update < MOTOR_UPDATE_RATE / 1000.0:
                time.sleep(0.001)
                continue
                
            last_motor_update = current_time

            try:
                # =============================================================================
                # UWB NAVIGATION LOGIC - FROM NEW CODE (exact preservation)
                # =============================================================================
                if self.state in [AGVState.UWB_NAVIGATION, AGVState.TASK1_NAVIGATION] and self.navigation:
                    nav_state = self.navigation.update()
                    
                    if nav_state == NavigationState.REACHED_TARGET:
                        self.state = AGVState.IDLE
                        self.desired_rpm = [0, 0]
                        if self.state == AGVState.TASK1_NAVIGATION:
                            logger.info("🎯 Task 1 completed successfully!")
                        else:
                            logger.info("🎯 Navigation target reached!")

                # =============================================================================
                # LINE FOLLOWING LOGIC - FROM OLD CODE (EXACT PRESERVATION)
                # =============================================================================
                elif self.state == AGVState.LINE_FOLLOW:
                    try:
                        median_value, position_value = self.sensor_queue.get_nowait()
                        
                        if median_value is not None and position_value is not None:
                            # Calculate line position - FROM OLD CODE
                            line_position, line_found, debug_info = self._calculate_line_position(position_value)
                            
                            if line_found:
                                # Line detected - calculate correction - FROM OLD CODE
                                speeds, error, correction = self._calculate_correction(line_position)
                                self.desired_rpm = speeds
                                self.pid_error = error
                                self.last_known_position = line_position
                                self.line_lost_timer = 0
                                self.line_detected = True
                                
                                logger.debug(f"Line at {line_position:.2f}, Error: {error:.2f}, Correction: {correction:.2f}, Speeds: L={speeds[0]:.1f}, R={speeds[1]:.1f}")
                                
                            else:
                                # Line lost - handle gracefully - FROM OLD CODE
                                if self.line_lost_timer == 0:
                                    self.line_lost_timer = current_time
                                
                                if current_time - self.line_lost_timer > LINE_LOST_TIMEOUT:
                                    self.desired_rpm = [0, 0]
                                    self.state = AGVState.LINE_LOST
                                    logger.warning(f"Line lost! {debug_info}")
                                else:
                                    logger.debug(f"Line temporarily lost: {debug_info}")
                                
                    except queue.Empty:
                        pass
                
                elif self.state == AGVState.LINE_LOST:
                    # Simple search pattern - FROM OLD CODE
                    self.desired_rpm = [5, 5]  # Slow turn right to search
                
                # =============================================================================
                # MOTOR COMMANDS - FROM OLD CODE (reliable communication)
                # =============================================================================
                with self.lock:
                    if self.motor_control.ser and self.motor_control.ser.is_open:
                        self.motor_control.send_rpm(MOTOR_ID_LEFT, self.desired_rpm[0])
                        self.motor_control.send_rpm(MOTOR_ID_RIGHT, self.desired_rpm[1])

            except Exception as e:
                logger.error(f"Critical error in control loop: {e}")
                self.reset_system_state()
                time.sleep(1)

    # =============================================================================
    # GETTER METHODS FOR EXTERNAL ACCESS
    # =============================================================================
    def get_uwb_position(self) -> tuple:
        """Get current UWB position - FROM NEW CODE"""
        with self.lock:
            return self.current_uwb_position

    def get_uwb_diagnostics(self) -> dict:
        """Get UWB positioning diagnostics - FROM NEW CODE"""
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
        """Reset UWB diagnostic counters - FROM NEW CODE"""
        with self.lock:
            self.uwb_update_count = 0
            self.uwb_error_count = 0
            self.position_accuracy_log.clear()
        
        if self.uwb:
            self.uwb.reset_position_filter()
        
        logger.info("UWB diagnostics reset")