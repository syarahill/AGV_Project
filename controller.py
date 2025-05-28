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
    from navigation import Navigation, NavigationState
except ImportError as e:
    logging.warning("UWB modules not found - continuing without UWB support")
    UWBReader = None
    Navigation = None
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
        
        # Initialize UWB if enabled
        self.uwb = None
        self.navigation = None
        self.uwb_enabled = enable_uwb
        
        if enable_uwb and UWBReader and Navigation:
            try:
                self.uwb = UWBReader(UWB_PORT)
                if self.uwb.connect():
                    self.navigation = Navigation(self.uwb, self.motor_control, self.sensor)
                    logger.info("UWB navigation initialized with enhanced positioning")
                    
                    # Log UWB configuration
                    if UWB_KALMAN_ENABLED:
                        logger.info(f"Kalman filtering enabled - Process noise: X={UWB_KALMAN_PROCESS_NOISE_X}, "
                                  f"Y={UWB_KALMAN_PROCESS_NOISE_Y}, Measurement noise: {UWB_KALMAN_MEASUREMENT_NOISE}")
                    else:
                        logger.info("Kalman filtering disabled")
                        
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
        
        # Line following improvements
        self.line_position_history = deque(maxlen=MOVING_AVERAGE_WINDOW)
        self.line_lost_timer = 0
        self.last_known_position = IDEAL_CENTER
        self.line_detected = False
        
        # UWB position tracking and quality monitoring
        self.current_uwb_position = (0.0, 0.0, 0.0)
        self.uwb_position_quality = {}
        self.uwb_last_update = 0
        
        # UWB diagnostics
        self.uwb_update_count = 0
        self.uwb_error_count = 0
        self.position_accuracy_log = deque(maxlen=100)  # Store last 100 position updates

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

    def _sensor_loop(self):
        if not self.sensor.connect():
            self.state = AGVState.ERROR
            return
        while self.running:
            data = self.sensor.read_data()
            if data:
                self.sensor_queue.put(data)
            time.sleep(SENSOR_READ_RATE / 1000.0)

    def _uwb_loop(self):
        """Continuously read UWB position with enhanced monitoring"""
        if not self.uwb:
            return
            
        self.uwb.continuous_read()  # Start the UWB continuous reading
        
        # Monitor the position updates
        while self.running and self.uwb:
            try:
                with self.lock:
                    new_position = self.uwb.get_current_position()
                    
                    # Check if position actually updated
                    if new_position != self.current_uwb_position:
                        self.current_uwb_position = new_position
                        self.uwb_last_update = time.time()
                        self.uwb_update_count += 1
                        
                        # Get position quality metrics
                        self.uwb_position_quality = self.uwb.get_position_quality()
                        
                        # Log position accuracy for diagnostics
                        if hasattr(self.uwb_position_quality, 'raw_position') and hasattr(self.uwb_position_quality, 'filtered_position'):
                            raw_pos = self.uwb_position_quality['raw_position']
                            filtered_pos = self.uwb_position_quality['filtered_position']
                            if raw_pos and filtered_pos:
                                accuracy = ((raw_pos[0] - filtered_pos[0])**2 + (raw_pos[1] - filtered_pos[1])**2)**0.5
                                self.position_accuracy_log.append(accuracy)
                        
                time.sleep(0.05)  # 20Hz update rate for monitoring
                
            except Exception as e:
                logger.error(f"UWB loop error: {e}")
                self.uwb_error_count += 1
                time.sleep(0.1)

    def _uwb_diagnostics_loop(self):
        """Background thread for UWB diagnostics and health monitoring"""
        if not self.uwb:
            return
            
        last_diagnostic_time = time.time()
        
        while self.running and self.uwb:
            current_time = time.time()
            
            # Run diagnostics every 10 seconds
            if current_time - last_diagnostic_time >= 10:
                try:
                    with self.lock:
                        quality = self.uwb_position_quality
                        
                        # Calculate average position accuracy if we have data
                        avg_accuracy = 0
                        if self.position_accuracy_log:
                            avg_accuracy = sum(self.position_accuracy_log) / len(self.position_accuracy_log)
                        
                        # Check for position timeout
                        time_since_update = current_time - self.uwb_last_update
                        
                        # Log diagnostic information
                        logger.info(f"UWB Diagnostics - Updates: {self.uwb_update_count}, "
                                  f"Errors: {self.uwb_error_count}, "
                                  f"Last update: {time_since_update:.1f}s ago, "
                                  f"Avg accuracy: {avg_accuracy:.3f}m")
                        
                        if quality:
                            logger.info(f"Position quality - Jump count: {quality.get('position_jump_count', 0)}, "
                                      f"Trilateration errors: {quality.get('trilateration_error_count', 0)}, "
                                      f"Kalman enabled: {quality.get('kalman_enabled', False)}")
                        
                        # Reset position filter if too many errors
                        if (quality.get('position_jump_count', 0) > 5 or 
                            quality.get('trilateration_error_count', 0) > 20):
                            logger.warning("High error count detected, resetting UWB position filter")
                            self.uwb.reset_position_filter()
                        
                        # Warning if no recent updates
                        if time_since_update > 5:
                            logger.warning(f"No UWB position updates for {time_since_update:.1f} seconds")
                    
                    last_diagnostic_time = current_time
                    
                except Exception as e:
                    logger.error(f"UWB diagnostics error: {e}")
            
            time.sleep(1)  # Check every second

    def navigate_to_position(self, x: float, y: float):
        """Navigate to a specific position using UWB"""
        if not self.navigation:
            logger.error("UWB navigation not available")
            return False
            
        self.navigation.set_target(x, y)
        self.state = AGVState.UWB_NAVIGATION
        logger.info(f"Starting navigation to ({x}, {y}) with enhanced positioning")
        return True

    def get_uwb_position(self) -> tuple:
        """Get current UWB position"""
        with self.lock:
            return self.current_uwb_position

    def get_uwb_diagnostics(self) -> dict:
        """Get UWB positioning diagnostics"""
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
        """Calculate line position with corrected logic"""
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
        """Calculate steering correction based on line position"""
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

    def _control_loop(self):
        last_motor_update = 0
        while self.running:
            current_time = time.time()
            
            # Process commands without blocking
            try:
                while not self.command_queue.empty():
                    cmd = self.command_queue.get_nowait()
                    if cmd[0] == "set_state":
                        new_state = cmd[1]
                        if new_state != self.state:
                            logger.info(f"State change: {self.state.name} -> {new_state.name}")
                            if new_state == AGVState.LINE_FOLLOW:
                                self.pid.reset()  # Reset PID when starting line follow
                            self.state = new_state
                    elif cmd[0] == "set_speed":
                        self.desired_rpm = cmd[1]
                    elif cmd[0] == "update_pid":
                        self.pid.kp, self.pid.ki, self.pid.kd = cmd[1]
                        logger.info(f"PID updated: Kp={self.pid.kp}, Ki={self.pid.ki}, Kd={self.pid.kd}")
                    elif cmd[0] == "navigate_to":
                        x, y = cmd[1]
                        self.navigate_to_position(x, y)
                    elif cmd[0] == "reset_uwb":
                        self.reset_uwb_diagnostics()
            except queue.Empty:
                pass

            # Only update motors at specified rate to reduce lag
            if current_time - last_motor_update < MOTOR_UPDATE_RATE / 1000.0:
                time.sleep(0.001)
                continue
                
            last_motor_update = current_time

            # UWB Navigation logic
            if self.state == AGVState.UWB_NAVIGATION and self.navigation:
                # Let navigation system handle heading detection
                nav_state = self.navigation.update()
                
                # Switch to line following when navigation requests it
                if nav_state == NavigationState.FOLLOWING_LINE:
                    self.state = AGVState.LINE_FOLLOW
                    self.pid.reset()
                    logger.info("Switched from UWB navigation to line following")
                elif nav_state == NavigationState.REACHED_TARGET:
                    self.state = AGVState.IDLE
                    self.desired_rpm = [0, 0]
                    logger.info("Target reached!")

            # Line following logic
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
                                # Stop or search for line
                                self.desired_rpm = [0, 0]
                                self.state = AGVState.LINE_LOST
                                logger.warning(f"Line lost! {debug_info}")
                            
                except queue.Empty:
                    pass
            
            elif self.state == AGVState.LINE_LOST:
                # Simple search pattern - slowly turn to find line again
                self.desired_rpm = [5, 5]  # Slow turn right to search
                
            # Send motor commands only if changed
            with self.lock:
                if self.motor_control.ser and self.motor_control.ser.is_open:
                    self.motor_control.send_rpm(MOTOR_ID_LEFT, self.desired_rpm[0])
                    self.motor_control.send_rpm(MOTOR_ID_RIGHT, self.desired_rpm[1])