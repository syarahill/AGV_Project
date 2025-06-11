import threading
import queue
import time
import logging
import math
from collections import deque
from enum import Enum
from config import *
from sensor_reader import SensorReader
from motor_control import MotorControl

# Try to import UWB modules, but continue if they don't exist yet
try:
    from uwb_reader import UWBReader
except ImportError as e:
    logging.warning("UWB modules not found - continuing without UWB support")
    UWBReader = None

# Import trajectory logger with detailed error logging
try:
    from trajectory_logger import TrajectoryLogger
except ImportError as e:
    logging.warning(f"Failed to import TrajectoryLogger: {e}")
    TrajectoryLogger = None

logger = logging.getLogger("Controller")

# Navigation sub-states for grid-based movement
class NavigationState(Enum):
    IDLE = 0
    CALCULATE_PATH = 1
    TURN_TO_HEADING = 2
    MOVE_FORWARD = 3
    CHECK_POSITION = 4
    REACHED_TARGET = 5
    ERROR = 6

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
        self.uwb_enabled = enable_uwb
        
        if enable_uwb and UWBReader:
            try:
                self.uwb = UWBReader(UWB_PORT)
                if self.uwb.connect():
                    logger.info("UWB navigation initialized")
                else:
                    logger.warning("Failed to initialize UWB - continuing without UWB")
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
        
        # Line following variables (FROM OLD CODE)
        self.line_position_history = deque(maxlen=MOVING_AVERAGE_WINDOW)
        self.line_lost_timer = 0
        self.last_known_position = IDEAL_CENTER
        self.line_detected = False
        
        # UWB position tracking
        self.current_uwb_position = (0.0, 0.0, 0.0)
        self.uwb_position_quality = {}
        self.uwb_last_update = 0
        
        # UWB diagnostics
        self.uwb_update_count = 0
        self.uwb_error_count = 0
        self.position_accuracy_log = deque(maxlen=50)
        
        # Navigation variables
        self.target_position = None
        self.navigation_start_time = 0
        
        # Grid navigation variables
        self.nav_state = NavigationState.IDLE
        self.nav_path = []  # List of waypoints [(x1,y1), (x2,y2), ...]
        self.current_waypoint_index = 0
        self.target_heading = 0
        self.turn_start_time = 0
        self.move_start_time = 0
        self.move_start_position = (0, 0)
        
        # Heading tracking
        self.current_heading = 0.0  # Always start facing North (Y+)
        self.wheel_base = 0.3  # Distance between wheels in meters (adjust based on your robot)
        self.wheel_circumference = 0.2  # Wheel circumference in meters (adjust based on your wheels)
        self.last_encoder_left = 0
        self.last_encoder_right = 0
        
        # Navigation parameters
        self.heading_tolerance = 5.0  # degrees
        self.position_tolerance = 0.15  # meters
        self.turn_timeout = 5.0  # seconds
        self.move_timeout = 10.0  # seconds
        
        # Trajectory logging
        self.trajectory_logger = None
        self.logging_active = False

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
        # Stop trajectory logging if active
        if self.trajectory_logger:
            self.stop_trajectory_logging()

    def reset_system_state(self):
        """Reset system to safe state on errors"""
        self.motor_control.emergency_stop()
        self.state = AGVState.IDLE
        self.nav_state = NavigationState.IDLE
        self.target_position = None
        if self.uwb:
            self.uwb.reset_position_filter()
        logger.info("System state reset to safe mode")

    def start_trajectory_logging(self):
        """Start logging trajectory data"""
        if TrajectoryLogger:
            self.trajectory_logger = TrajectoryLogger()
            self.logging_active = True
            logger.info("Trajectory logging started")
        else:
            logger.warning("TrajectoryLogger not available")

    def stop_trajectory_logging(self):
        """Stop logging and save data"""
        if self.trajectory_logger:
            self.trajectory_logger.finalize()
            self.trajectory_logger = None
            self.logging_active = False
            logger.info("Trajectory logging stopped")

    def update_heading_from_encoders(self, encoder_left, encoder_right):
        """Update heading based on encoder differences when turning"""
        if self.last_encoder_left == 0 and self.last_encoder_right == 0:
            self.last_encoder_left = encoder_left
            self.last_encoder_right = encoder_right
            return
        
        # Calculate distance traveled by each wheel
        left_distance = (encoder_left - self.last_encoder_left) * self.wheel_circumference / 360.0
        right_distance = (encoder_right - self.last_encoder_right) * self.wheel_circumference / 360.0
        
        # Calculate heading change
        distance_diff = right_distance - left_distance
        heading_change = (distance_diff / self.wheel_base) * (180.0 / math.pi)
        
        # Update heading
        self.current_heading = (self.current_heading + heading_change) % 360
        
        # Store current encoder values
        self.last_encoder_left = encoder_left
        self.last_encoder_right = encoder_right

    def normalize_angle(self, angle):
        """Normalize angle to 0-360 range"""
        while angle < 0:
            angle += 360
        while angle >= 360:
            angle -= 360
        return angle

    def calculate_turn_direction(self, current, target):
        """Calculate shortest turn direction and angle"""
        diff = self.normalize_angle(target - current)
        if diff > 180:
            return -(360 - diff)  # Turn left (negative)
        else:
            return diff  # Turn right (positive)

    def calculate_grid_path(self, start_x, start_y, target_x, target_y):
        """Calculate simple Manhattan path along grid lines"""
        path = []
        current_x, current_y = start_x, start_y
        
        # Convert to grid coordinates
        start_grid_x = int(start_x / CELL_SIZE)
        start_grid_y = int(start_y / CELL_SIZE)
        target_grid_x = int(target_x / CELL_SIZE)
        target_grid_y = int(target_y / CELL_SIZE)
        
        # Add starting position
        path.append((current_x, current_y))
        
        # Decide whether to move X first or Y first based on current heading
        # This minimizes the number of turns needed
        dx = target_grid_x - start_grid_x
        dy = target_grid_y - start_grid_y
        
        if self.current_heading == 0 or self.current_heading == 180:  # Facing North/South
            # Move Y first if we need to go that direction
            if dy != 0:
                # Move to target Y
                path.append((current_x, target_y))
                current_y = target_y
            if dx != 0:
                # Then move to target X
                path.append((target_x, current_y))
        else:  # Facing East/West or other
            # Move X first if we need to go that direction
            if dx != 0:
                # Move to target X
                path.append((target_x, current_y))
                current_x = target_x
            if dy != 0:
                # Then move to target Y
                path.append((current_x, target_y))
        
        logger.info(f"Grid path calculated: {path}")
        return path

    def get_heading_for_movement(self, from_x, from_y, to_x, to_y):
        """Calculate required heading to move from one point to another"""
        dx = to_x - from_x
        dy = to_y - from_y
        
        # Determine cardinal direction
        if abs(dx) > abs(dy):  # Moving primarily in X
            if dx > 0:
                return 90  # East
            else:
                return 270  # West
        else:  # Moving primarily in Y
            if dy > 0:
                return 0  # North
            else:
                return 180  # South

    def navigate_to_position(self, x: float, y: float):
        """Navigate to a specific position using UWB and grid-based movement"""
        if not self.uwb_enabled or not self.uwb:
            logger.error("UWB navigation not available")
            return False
            
        if not (0 <= x <= ROOM_WIDTH and 0 <= y <= ROOM_HEIGHT):
            logger.error(f"Target ({x}, {y}) outside room bounds")
            return False
        
        # Get current position
        current_x, current_y, _ = self.get_uwb_position()
        
        # Calculate grid path
        self.nav_path = self.calculate_grid_path(current_x, current_y, x, y)
        self.current_waypoint_index = 0
        
        # Set navigation state
        self.target_position = (x, y)
        self.navigation_start_time = time.time()
        self.state = AGVState.UWB_NAVIGATION
        self.nav_state = NavigationState.CALCULATE_PATH
        
        logger.info(f"Starting grid navigation to ({x:.2f}, {y:.2f})")
        logger.info(f"Current heading: {self.current_heading:.1f}°")
        return True

    def start_task1(self):
        """Start Task 1 - Move to grid position (6,10)"""
        target_x, target_y = TASK1_TARGET_GRID
        real_x = (target_x + 0.5) * CELL_SIZE
        real_y = (target_y + 0.5) * CELL_SIZE
        return self.navigate_to_position(real_x, real_y)

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
                'kalman_enabled': UWB_KALMAN_ENABLED,
                'current_heading': self.current_heading
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

    def _sensor_loop(self):
        """RESTORED FROM OLD CODE: Original sensor loop timing"""
        if not self.sensor.connect():
            self.state = AGVState.ERROR
            return
        while self.running:
            data = self.sensor.read_data()
            if data:
                self.sensor_queue.put(data)
            time.sleep(SENSOR_READ_RATE / 1000.0)  # Back to 25ms

    def _uwb_loop(self):
        """UWB position monitoring - UNCHANGED"""
        if not self.uwb:
            return
            
        self.uwb.continuous_read()
        
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
                
                time.sleep(0.03)
                
            except Exception as e:
                logger.error(f"UWB loop error: {e}")
                self.uwb_error_count += 1
                if self.uwb_error_count > 10:
                    self.reset_system_state()
                    self.uwb_error_count = 0
                time.sleep(0.1)

    def _uwb_diagnostics_loop(self):
        """UWB diagnostics - UNCHANGED"""
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
                                   f"Avg accuracy: {avg_accuracy:.3f}m, "
                                   f"Current heading: {self.current_heading:.1f}°")
                        
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

    def _calculate_line_position(self, position_value):
        """RESTORED FROM OLD CODE: Original calculation logic"""
        active_sensors = [i + 1 for i in range(16) if not (position_value & (1 << i))]
        
        if not active_sensors:
            return None, False, "No line detected"
            
        if len(active_sensors) > MAX_SENSORS_FOR_LINE:
            logger.warning(f"Too many sensors active ({len(active_sensors)}), possible noise")
            return None, False, f"Too many sensors: {active_sensors}"
            
        if len(active_sensors) >= MIN_SENSORS_FOR_LINE:
            line_position = sum(active_sensors) / len(active_sensors)
            self.line_position_history.append(line_position)
            smoothed_position = sum(self.line_position_history) / len(self.line_position_history)
            
            right_side_active = [s for s in active_sensors if s in LOW_SENSORS]
            left_side_active = [s for s in active_sensors if s in HIGH_SENSORS]
            
            side_info = f"Right:{right_side_active}, Left:{left_side_active}"
            logger.debug(f"Active sensors: {active_sensors}, Position: {smoothed_position:.2f}, {side_info}")
            
            return smoothed_position, True, side_info
            
        return None, False, f"Not enough sensors: {active_sensors}"

    def _calculate_correction(self, line_position):
        """RESTORED FROM OLD CODE: Dead zone removed for better control"""
        error = line_position - IDEAL_CENTER
        
        current_time = time.time()
        dt = current_time - self.pid.last_time
        correction = self.pid.compute(error, dt)
        
        # Limit correction
        if correction > MAX_CORRECTION:
            correction = MAX_CORRECTION
        elif correction < -MAX_CORRECTION:
            correction = -MAX_CORRECTION
            
        left_speed = MAX_SPEED - correction
        right_speed = -(MAX_SPEED + correction)
        
        return [left_speed, right_speed], error, correction

    def process_grid_navigation(self):
        """Process grid-based navigation state machine"""
        current_time = time.time()
        current_x, current_y, _ = self.get_uwb_position()
        
        if self.nav_state == NavigationState.CALCULATE_PATH:
            # Path already calculated in navigate_to_position
            if len(self.nav_path) > 1:
                self.current_waypoint_index = 1  # Skip starting position
                self.nav_state = NavigationState.TURN_TO_HEADING
                logger.info(f"Starting navigation with {len(self.nav_path)} waypoints")
            else:
                logger.info("Already at target!")
                self.nav_state = NavigationState.REACHED_TARGET
                
        elif self.nav_state == NavigationState.TURN_TO_HEADING:
            # Calculate required heading for next waypoint
            if self.current_waypoint_index < len(self.nav_path):
                target_waypoint = self.nav_path[self.current_waypoint_index]
                self.target_heading = self.get_heading_for_movement(
                    current_x, current_y, target_waypoint[0], target_waypoint[1]
                )
                
                # Calculate turn angle
                turn_angle = self.calculate_turn_direction(self.current_heading, self.target_heading)
                
                if abs(turn_angle) < self.heading_tolerance:
                    # Already facing the right direction
                    self.desired_rpm = [0, 0]
                    self.nav_state = NavigationState.MOVE_FORWARD
                    self.move_start_time = current_time
                    self.move_start_position = (current_x, current_y)
                    logger.info(f"Heading aligned, moving to waypoint {self.current_waypoint_index}")
                else:
                    # Need to turn
                    if self.turn_start_time == 0:
                        self.turn_start_time = current_time
                        logger.info(f"Turning from {self.current_heading:.1f}° to {self.target_heading:.1f}° (angle: {turn_angle:.1f}°)")
                    
                    # Check timeout
                    if current_time - self.turn_start_time > self.turn_timeout:
                        logger.warning("Turn timeout!")
                        self.nav_state = NavigationState.ERROR
                    else:
                        # Apply turn speed
                        turn_speed = NAVIGATION_TURN_SPEED
                        if turn_angle > 0:  # Turn right
                            self.desired_rpm = [turn_speed, turn_speed]
                        else:  # Turn left
                            self.desired_rpm = [-turn_speed, -turn_speed]
                        
                        # Update heading based on encoders (would need encoder feedback)
                        # For now, estimate based on time
                        turn_rate = 45  # degrees per second (estimate)
                        estimated_turn = turn_rate * (current_time - self.turn_start_time)
                        if turn_angle > 0:
                            self.current_heading = self.normalize_angle(self.current_heading + estimated_turn)
                        else:
                            self.current_heading = self.normalize_angle(self.current_heading - estimated_turn)
                            
        elif self.nav_state == NavigationState.MOVE_FORWARD:
            target_waypoint = self.nav_path[self.current_waypoint_index]
            distance_to_waypoint = math.sqrt(
                (target_waypoint[0] - current_x)**2 + 
                (target_waypoint[1] - current_y)**2
            )
            
            if distance_to_waypoint < self.position_tolerance:
                # Reached waypoint
                self.desired_rpm = [0, 0]
                self.current_waypoint_index += 1
                
                if self.current_waypoint_index >= len(self.nav_path):
                    # Reached final target
                    self.nav_state = NavigationState.REACHED_TARGET
                    logger.info("🎯 Target reached!")
                else:
                    # Move to next waypoint
                    self.nav_state = NavigationState.TURN_TO_HEADING
                    self.turn_start_time = 0
                    logger.info(f"Waypoint {self.current_waypoint_index} reached, moving to next")
            else:
                # Check timeout
                if current_time - self.move_start_time > self.move_timeout:
                    logger.warning("Move timeout!")
                    self.nav_state = NavigationState.ERROR
                else:
                    # Continue moving forward
                    self.desired_rpm = [-NAVIGATION_MOVE_SPEED, NAVIGATION_MOVE_SPEED]
                    
                    # Log progress
                    if int(current_time) % 2 == 0:  # Every 2 seconds
                        logger.debug(f"Moving: {distance_to_waypoint:.2f}m to waypoint")
                        
        elif self.nav_state == NavigationState.REACHED_TARGET:
            self.desired_rpm = [0, 0]
            self.state = AGVState.IDLE
            self.nav_state = NavigationState.IDLE
            logger.info("Navigation complete!")
            
        elif self.nav_state == NavigationState.ERROR:
            self.desired_rpm = [0, 0]
            self.state = AGVState.IDLE
            self.nav_state = NavigationState.IDLE
            logger.error("Navigation error - stopping")

    def _control_loop(self):
        """RESTORED: Simplified control loop without rate limiting for line following"""
        while self.running:
            current_time = time.time()
            
            # Process commands
            try:
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
                elif cmd[0] == "start_trajectory_logging":
                    self.start_trajectory_logging()
                elif cmd[0] == "stop_trajectory_logging":
                    self.stop_trajectory_logging()
            except queue.Empty:
                pass

            # UWB Navigation logic - Updated with grid navigation
            if self.state == AGVState.UWB_NAVIGATION:
                self.process_grid_navigation()

            # RESTORED FROM OLD CODE: Line following logic with trajectory logging
            elif self.state == AGVState.LINE_FOLLOW:
                try:
                    median_value, position_value = self.sensor_queue.get_nowait()
                    
                    line_position, line_found, debug_info = self._calculate_line_position(position_value)
                    
                    if line_found:
                        speeds, error, correction = self._calculate_correction(line_position)
                        self.desired_rpm = speeds
                        self.pid_error = error
                        self.last_known_position = line_position
                        self.line_lost_timer = 0
                        self.line_detected = True
                        
                        logger.debug(f"Line at {line_position:.2f}, Error: {error:.2f}, Correction: {correction:.2f}, Speeds: L={speeds[0]:.1f}, R={speeds[1]:.1f}")
                        
                        # Update trajectory logger if active
                        if self.logging_active and self.trajectory_logger:
                            dt = current_time - self.pid.last_time
                            
                            # Calculate PID components
                            pid_components = {
                                'P': self.pid.kp * error,
                                'I': self.pid.ki * self.pid.integral,
                                'D': self.pid.kd * (error - self.pid.last_error) / dt if dt > 0 else 0
                            }
                            
                            self.trajectory_logger.update(
                                line_position=line_position,
                                raw_sensor_value=position_value,
                                pid_error=error,
                                pid_correction=correction,
                                left_speed=speeds[0],
                                right_speed=speeds[1],
                                pid_components=pid_components,
                                line_detected=True,
                                dt=dt
                            )
                        
                    else:
                        if self.line_lost_timer == 0:
                            self.line_lost_timer = current_time
                        
                        # Update trajectory logger for line lost
                        if self.logging_active and self.trajectory_logger:
                            self.trajectory_logger.update(
                                line_position=None,
                                raw_sensor_value=position_value,
                                pid_error=0,
                                pid_correction=0,
                                left_speed=self.desired_rpm[0],
                                right_speed=self.desired_rpm[1],
                                line_detected=False,
                                dt=MOTOR_UPDATE_RATE / 1000.0
                            )
                        
                        if current_time - self.line_lost_timer > LINE_LOST_TIMEOUT:
                            self.desired_rpm = [0, 0]
                            self.state = AGVState.LINE_LOST
                            logger.warning(f"Line lost! {debug_info}")
                        else:
                            logger.debug(f"Line temporarily lost: {debug_info}")
                            
                except queue.Empty:
                    pass
            
            elif self.state == AGVState.LINE_LOST:
                # RESTORED FROM OLD CODE
                self.desired_rpm = [5, 5]
                
            # Send motor commands - RESTORED WITHOUT RATE LIMITING
            with self.lock:
                self.motor_control.send_rpm(MOTOR_ID_LEFT, self.desired_rpm[0])
                self.motor_control.send_rpm(MOTOR_ID_RIGHT, self.desired_rpm[1])
                
            time.sleep(MOTOR_UPDATE_RATE / 1000.0)  # Back to 50ms