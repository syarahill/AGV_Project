import threading
import queue
import time
import logging
import math
from collections import deque
from config import *
from sensor_reader import SensorReader
from motor_control import MotorControl
from line_following import LineFollowing
from uwb_navigation import UWBNavigation

# Try to import UWB modules
try:
    from uwb_reader import UWBReader
except ImportError as e:
    logging.warning("UWB modules not found - continuing without UWB support")
    UWBReader = None

# Import trajectory logger
try:
    from trajectory_logger import TrajectoryLogger
except ImportError as e:
    logging.warning(f"Failed to import TrajectoryLogger: {e}")
    TrajectoryLogger = None

logger = logging.getLogger("Controller")

class Controller:
    def __init__(self, enable_uwb=True):
        self.sensor = SensorReader()
        self.motor_control = MotorControl()
        
        # Initialize mission modules
        self.line_following = LineFollowing(
            self.sensor,
            self.motor_control,
            None  # Don't create trajectory logger yet
        )
        
        # Initialize trajectory logger (but not active)
        self.trajectory_logger = None
        if TrajectoryLogger:
            self.trajectory_logger = TrajectoryLogger()
            self.trajectory_logger.logging_active = False  # Start with logging OFF
            self.line_following.set_trajectory_logger(self.trajectory_logger)
        
        # Initialize UWB
        self.uwb = None
        self.uwb_enabled = enable_uwb
        self.uwb_navigation = None
        
        # UWB position tracking
        self.current_uwb_position = (0.0, 0.0, 0.0)
        self.uwb_position_quality = {}
        self.uwb_last_update = 0
        
        # UWB diagnostics
        self.uwb_update_count = 0
        self.uwb_error_count = 0
        self.position_accuracy_log = deque(maxlen=50)
        
        if enable_uwb and UWBReader:
            try:
                self.uwb = UWBReader(UWB_PORT)
                if self.uwb.connect():
                    self.uwb.continuous_read()
                    self.uwb_navigation = UWBNavigation(self.motor_control, self.uwb)
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
        self.desired_rpm = [0, 0]
        self.lock = threading.Lock()
        self.running = False

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
        if self.trajectory_logger and self.trajectory_logger.logging_active:
            self.stop_trajectory_logging()

    def reset_system_state(self):
        """Reset system to safe state on errors"""
        self.motor_control.emergency_stop()
        self.state = AGVState.IDLE
        if self.uwb_navigation:
            self.uwb_navigation.reset()
        logger.info("System state reset to safe mode")

    def start_trajectory_logging(self):
        """Start logging trajectory data"""
        if self.trajectory_logger and not self.trajectory_logger.logging_active:
            self.trajectory_logger.logging_active = True
            logger.info("Trajectory logging started")
        else:
            logger.warning("Trajectory logger not available or already active")

    def stop_trajectory_logging(self):
        """Stop logging and save data"""
        if self.trajectory_logger and self.trajectory_logger.logging_active:
            self.trajectory_logger.logging_active = False
            self.trajectory_logger.finalize()
            logger.info("Trajectory logging stopped and saved")

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
                'current_heading': self.uwb_navigation.current_heading if self.uwb_navigation else 0
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

    def get_current_grid(self) -> tuple:
        """Get current grid position during navigation"""
        if self.state == AGVState.UWB_NAVIGATION and self.uwb_navigation:
            return self.uwb_navigation.get_current_grid()
        return (0, 0)

    def _sensor_loop(self):
        """Sensor reading loop"""
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
            
        while self.running and self.uwb:
            try:
                with self.lock:
                    new_position = self.uwb.get_current_position()
                    
                    if new_position != self.current_uwb_position:
                        self.current_uwb_position = new_position
                        self.uwb_last_update = time.time()
                        self.uwb_update_count += 1
                        
                        self.uwb_position_quality = self.uwb.get_position_quality()
                        
                        if self.uwb_position_quality: 
                            raw_pos = self.uwb_position_quality.get('raw_position')
                            filtered_pos = self.uwb_position_quality.get('filtered_position')
                            if raw_pos and filtered_pos:
                                dx = raw_pos[0] - filtered_pos[0]
                                dy = raw_pos[1] - filtered_pos[1]
                                accuracy = math.sqrt(dx*dx + dy*dy)
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
                                   f"Current heading: {self.uwb_navigation.current_heading if self.uwb_navigation else 0:.1f}°")
                        
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

    def navigate_to_position(self, x: float, y: float):
        """Navigate to a specific position using UWB"""
        if not self.uwb_enabled or not self.uwb_navigation:
            logger.error("UWB navigation not available")
            return False
        
        # Set the state to UWB_NAVIGATION explicitly
        self.state = AGVState.UWB_NAVIGATION
        return self.uwb_navigation.navigate_to_position(x, y)

    def start_task1(self):
        """Start Task 1 - Move to grid position (6,10)"""
        target_x, target_y = TASK1_TARGET_GRID
        real_x = (target_x + 0.5) * CELL_SIZE
        real_y = (target_y + 0.5) * CELL_SIZE
        # Set the state to UWB_NAVIGATION explicitly
        self.state = AGVState.UWB_NAVIGATION
        return self.navigate_to_position(real_x, real_y)

    def _control_loop(self):
        """Main control loop"""
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
                            self.line_following.reset()
                        self.state = new_state
                elif cmd[0] == "set_speed":
                    self.desired_rpm = cmd[1]
                elif cmd[0] == "update_pid":
                    kp, ki, kd = cmd[1]
                    self.line_following.pid.kp = kp
                    self.line_following.pid.ki = ki
                    self.line_following.pid.kd = kd
                    logger.info(f"PID updated: Kp={kp}, Ki={ki}, Kd={kd}")
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

            # UWB Navigation
            if self.state == AGVState.UWB_NAVIGATION and self.uwb_navigation:
                navigation_result = self.uwb_navigation.update(current_time)
                self.uwb_navigation.apply_motor_commands()
                
                if navigation_result is True:
                    self.state = AGVState.IDLE
                elif navigation_result is False:
                    self.state = AGVState.ERROR

            # Line following
            elif self.state == AGVState.LINE_FOLLOW:
                try:
                    sensor_data = self.sensor_queue.get_nowait()
                    line_found = self.line_following.update(sensor_data, current_time)
                    
                    if line_found:
                        # Line detected, apply PID control
                        self.line_following.apply_motor_commands()
                    else:
                        # Line lost, move forward slowly to find it
                        self.state = AGVState.LINE_LOST
                        
                except queue.Empty:
                    # No new sensor data, keep last motor commands
                    self.line_following.apply_motor_commands()
            
            # Line lost state - move forward slowly
            elif self.state == AGVState.LINE_LOST:
                self.motor_control.send_rpm(MOTOR_ID_LEFT, 5)
                self.motor_control.send_rpm(MOTOR_ID_RIGHT, -5)
                
                # Check if line is found again
                try:
                    sensor_data = self.sensor_queue.get_nowait()
                    # Just check if line is detected, don't update line following
                    median_value, position_value = sensor_data
                    active_sensors = [i + 1 for i in range(16) if not (position_value & (1 << i))]
                    
                    if len(active_sensors) >= MIN_SENSORS_FOR_LINE and len(active_sensors) <= MAX_SENSORS_FOR_LINE:
                        # Line found again
                        self.state = AGVState.LINE_FOLLOW
                        self.line_following.reset()
                        logger.info("Line found again!")
                except queue.Empty:
                    pass
            
            # Manual control and other states
            else:
                with self.lock:
                    self.motor_control.send_rpm(MOTOR_ID_LEFT, self.desired_rpm[0])
                    self.motor_control.send_rpm(MOTOR_ID_RIGHT, self.desired_rpm[1])
                
            time.sleep(MOTOR_UPDATE_RATE / 1000.0)  # Back to 50ms