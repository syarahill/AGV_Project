import threading
import queue
import time
import logging
from collections import deque
from config import *
from sensor_reader import SensorReader
from motor_control import MotorControl

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
    def __init__(self):
        self.sensor = SensorReader()
        self.motor_control = MotorControl()
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

    def start(self):
        self.running = True
        threading.Thread(target=self._sensor_loop, daemon=True).start()
        threading.Thread(target=self._control_loop, daemon=True).start()

    def stop(self):
        self.running = False
        self.sensor.close()
        self.motor_control.close()

    def _sensor_loop(self):
        if not self.sensor.connect():
            self.state = AGVState.ERROR
            return
        while self.running:
            data = self.sensor.read_data()
            if data:
                self.sensor_queue.put(data)
            time.sleep(SENSOR_READ_RATE / 1000.0)

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
                            self.pid.reset()  # Reset PID when starting line follow
                        self.state = new_state
                elif cmd[0] == "set_speed":
                    self.desired_rpm = cmd[1]
                elif cmd[0] == "update_pid":
                    self.pid.kp, self.pid.ki, self.pid.kd = cmd[1]
                    logger.info(f"PID updated: Kp={self.pid.kp}, Ki={self.pid.ki}, Kd={self.pid.kd}")
            except queue.Empty:
                pass

            # Line following logic
            if self.state == AGVState.LINE_FOLLOW:
                try:
                    median_value, position_value = self.sensor_queue.get_nowait()
                    
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
                        
                        logger.debug(f"Line at {line_position:.2f}, Error: {error:.2f}, Correction: {correction:.2f}, Speeds: L={speeds[0]:.1f}, R={speeds[1]:.1f}")
                        
                    else:
                        # Line lost - handle gracefully
                        self.line_lost_timer = current_time
                        self.line_detected = False
                        
                        if current_time - self.line_lost_timer > LINE_LOST_TIMEOUT:
                            # Stop or search for line
                            self.desired_rpm = [0, 0]
                            self.state = AGVState.LINE_LOST
                            logger.warning(f"Line lost! {debug_info}")
                        else:
                            # Continue with last known correction for a short time
                            logger.debug(f"Line temporarily lost: {debug_info}")
                            
                except queue.Empty:
                    pass
            
            elif self.state == AGVState.LINE_LOST:
                # Simple search pattern - slowly turn to find line again
                self.desired_rpm = [5, 5]  # Slow turn right to search
                
            # Send motor commands
            with self.lock:
                self.motor_control.send_rpm(MOTOR_ID_LEFT, self.desired_rpm[0])
                self.motor_control.send_rpm(MOTOR_ID_RIGHT, self.desired_rpm[1])
                
            time.sleep(MOTOR_UPDATE_RATE / 1000.0)