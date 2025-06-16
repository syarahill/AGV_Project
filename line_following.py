import math
import time
import logging
from collections import deque
from config import *

logger = logging.getLogger("LineFollowing")

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

class LineFollowing:
    def __init__(self, sensor_reader, motor_control, trajectory_logger=None):
        self.sensor_reader = sensor_reader
        self.motor_control = motor_control
        self.trajectory_logger = trajectory_logger  # Can be None
        
        self.pid = PID(INITIAL_PID_KP, INITIAL_PID_KI, INITIAL_PID_KD)
        self.line_position_history = deque(maxlen=MOVING_AVERAGE_WINDOW)
        self.line_lost_timer = 0
        self.last_known_position = IDEAL_CENTER
        self.line_detected = False
        self.pid_error = 0.0
        self.desired_rpm = [0, 0]

    def set_trajectory_logger(self, logger):
        """Set trajectory logger after initialization"""
        self.trajectory_logger = logger

    def _calculate_line_position(self, position_value):
        """Calculate line position from sensor data"""
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
        """Calculate motor correction based on line position"""
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

    def update(self, sensor_data, current_time):
        """Update line following state"""
        if not sensor_data:
            return
            
        median_value, position_value = sensor_data
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
            if self.trajectory_logger and self.trajectory_logger.logging_active:
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
            if self.trajectory_logger and self.trajectory_logger.logging_active:
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
                logger.warning(f"Line lost! {debug_info}")
                return False  # Line lost
            else:
                logger.debug(f"Line temporarily lost: {debug_info}")
                
        return True  # Line still detected

    def apply_motor_commands(self):
        """Apply calculated motor commands"""
        self.motor_control.send_rpm(MOTOR_ID_LEFT, self.desired_rpm[0])
        self.motor_control.send_rpm(MOTOR_ID_RIGHT, self.desired_rpm[1])

    def reset(self):
        """Reset line following state"""
        self.pid.reset()
        self.line_position_history.clear()
        self.line_lost_timer = 0
        self.last_known_position = IDEAL_CENTER
        self.line_detected = False
        self.pid_error = 0.0
        self.desired_rpm = [0, 0]