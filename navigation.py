import math
import logging
import time
from typing import Tuple, List, Optional
from enum import Enum
from config import *

logger = logging.getLogger("Navigation")

class NavigationState(Enum):
    IDLE = 0
    DETECTING_HEADING = 1
    MOVING_TO_TARGET = 2
    APPROACHING_LINE = 3
    FOLLOWING_LINE = 4
    REACHED_TARGET = 5

class PIDController:
    """OPTIMIZED: Fast PID controller for cross-track error correction"""
    def __init__(self, Kp=VIRTUAL_LINE_CROSS_TRACK_KP, Ki=VIRTUAL_LINE_CROSS_TRACK_KI, Kd=VIRTUAL_LINE_CROSS_TRACK_KD):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0
        self.max_integral = 30.0  # Reduced for faster response
        
    def update(self, error):
        self.integral += error
        # OPTIMIZED: Faster integral windup protection
        self.integral = max(-self.max_integral, min(self.max_integral, self.integral))
            
        derivative = error - self.prev_error
        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)
        self.prev_error = error
        return output
    
    def reset(self):
        self.prev_error = 0
        self.integral = 0

class Navigation:
    def __init__(self, uwb_reader, motor_control, sensor_reader=None):
        """OPTIMIZED: Initialize fast navigation system"""
        self.uwb = uwb_reader
        self.motor = motor_control
        self.sensor = sensor_reader
        
        # Navigation parameters
        self.target_position = None
        self.start_position = None
        self.target_threshold = TARGET_THRESHOLD
        self.state = NavigationState.IDLE
        
        # OPTIMIZED: Movement parameters
        self.base_speed = NAVIGATION_MOVE_SPEED
        self.turn_speed = NAVIGATION_TURN_SPEED
        
        # OPTIMIZED: Faster heading detection
        self.robot_heading = None
        self.heading_detection_start_pos = None
        self.heading_detection_start_time = None
        self.heading_detection_timeout = HEADING_DETECTION_TIMEOUT
        
        # OPTIMIZED: Fast virtual line following
        self.virtual_line_start = None
        self.virtual_line_end = None
        self.cross_track_pid = PIDController()
        
        # OPTIMIZED: Simplified control
        self.use_grid_mode = False
        self.line_approach_distance = LINE_APPROACH_DISTANCE
        
        # Room bounds
        self.room_width = ROOM_WIDTH
        self.room_height = ROOM_HEIGHT
        
        logger.info(f"Fast virtual line navigation initialized for {self.room_width}×{self.room_height}m room")
        
    def set_target(self, x: float, y: float):
        """OPTIMIZED: Set target position with fast validation"""
        # Quick bounds check
        if not (0 <= x <= self.room_width and 0 <= y <= self.room_height):
            logger.error(f"Target ({x}, {y}) is outside room bounds")
            return False
        
        current_pos = self.uwb.get_current_position()
        self.start_position = (current_pos[0], current_pos[1])
        self.target_position = (x, y)
        
        # Create virtual line from start to target
        self.virtual_line_start = self.start_position
        self.virtual_line_end = self.target_position
        
        # Reset navigation state
        self.state = NavigationState.DETECTING_HEADING
        self.robot_heading = None
        self.cross_track_pid.reset()
        self.use_grid_mode = False
        
        logger.info(f"🎯 Target set: {self.target_position}, Virtual line: {self.virtual_line_start} -> {self.virtual_line_end}")
        return True
        
    def detect_heading_by_movement(self) -> bool:
        """OPTIMIZED: Fast heading detection by movement"""
        current_pos = self.uwb.get_current_position()
        current_time = time.time()
        
        if self.heading_detection_start_pos is None:
            # Start heading detection
            self.heading_detection_start_pos = (current_pos[0], current_pos[1])
            self.heading_detection_start_time = current_time
            
            # Move forward to detect heading
            self.motor.send_rpm(1, -HEADING_DETECTION_SPEED)  # Left motor reverse (forward)
            self.motor.send_rpm(2, HEADING_DETECTION_SPEED)   # Right motor forward
            
            logger.debug(f"Starting heading detection from position: {self.heading_detection_start_pos}")
            return False
        
        # Check if we've moved enough to calculate heading
        dx = current_pos[0] - self.heading_detection_start_pos[0]
        dy = current_pos[1] - self.heading_detection_start_pos[1]
        distance_moved = math.sqrt(dx**2 + dy**2)
        
        # OPTIMIZED: Reduced distance requirement for faster detection
        if distance_moved >= HEADING_DETECTION_MIN_DISTANCE or (current_time - self.heading_detection_start_time) > self.heading_detection_timeout:
            # Stop motors
            self.motor.send_rpm(1, 0)
            self.motor.send_rpm(2, 0)
            
            if distance_moved >= 0.1:  # Minimum 10cm to calculate heading
                # Calculate actual heading from movement
                self.robot_heading = math.degrees(math.atan2(dy, dx))
                if self.robot_heading < 0:
                    self.robot_heading += 360
                    
                logger.info(f"🧭 Heading detected: {self.robot_heading:.1f}° (moved {distance_moved:.2f}m)")
                return True
            else:
                # Fallback to grid mode if can't detect heading
                logger.warning("Could not detect heading, switching to grid mode")
                self.use_grid_mode = True
                return True
                
        return False
    
    def calculate_cross_track_error(self, current_pos: Tuple[float, float]) -> float:
        """OPTIMIZED: Fast cross-track error calculation"""
        if not self.virtual_line_start or not self.virtual_line_end:
            return 0.0
        
        # Vector from start to end of virtual line
        line_dx = self.virtual_line_end[0] - self.virtual_line_start[0]
        line_dy = self.virtual_line_end[1] - self.virtual_line_start[1]
        line_length = math.sqrt(line_dx**2 + line_dy**2)
        
        if line_length < 0.01:
            return 0.0
        
        # Normalize line vector
        line_dx_norm = line_dx / line_length
        line_dy_norm = line_dy / line_length
        
        # Vector from line start to current position
        pos_dx = current_pos[0] - self.virtual_line_start[0]
        pos_dy = current_pos[1] - self.virtual_line_start[1]
        
        # Cross-track error (perpendicular distance to line)
        cross_track_error = (pos_dx * (-line_dy_norm)) + (pos_dy * line_dx_norm)
        
        return cross_track_error
    
    def get_distance_to_target(self) -> float:
        """OPTIMIZED: Fast distance calculation"""
        if not self.target_position:
            return float('inf')
            
        current_pos = self.uwb.get_current_position()
        return math.sqrt(
            (self.target_position[0] - current_pos[0])**2 + 
            (self.target_position[1] - current_pos[1])**2
        )
    
    def grid_based_movement(self, current_pos: Tuple[float, float]) -> bool:
        """OPTIMIZED: Fast grid-based movement fallback"""
        if not self.target_position:
            return False
        
        dx = self.target_position[0] - current_pos[0]
        dy = self.target_position[1] - current_pos[1]
        
        # OPTIMIZED: Simplified threshold check
        threshold = GRID_MOVEMENT_THRESHOLD
        
        # Prioritize X movement first, then Y
        if abs(dx) > threshold:
            if dx > 0:  # Go East
                self.motor.send_rpm(1, -self.base_speed)
                self.motor.send_rpm(2, self.base_speed)
            else:  # Go West
                self.motor.send_rpm(1, self.base_speed)
                self.motor.send_rpm(2, -self.base_speed)
        elif abs(dy) > threshold:
            if dy > 0:  # Go North
                self.motor.send_rpm(1, -self.base_speed)
                self.motor.send_rpm(2, self.base_speed)
            else:  # Go South
                self.motor.send_rpm(1, self.base_speed)
                self.motor.send_rpm(2, -self.base_speed)
        else:
            # Close enough to target
            self.motor.send_rpm(1, 0)
            self.motor.send_rpm(2, 0)
            return True
            
        return False
    
    def virtual_line_following(self, current_pos: Tuple[float, float]) -> bool:
        """OPTIMIZED: Fast virtual line following like magnetic line following"""
        # Calculate cross-track error (how far off the virtual line)
        cross_track_error = self.calculate_cross_track_error(current_pos)
        
        # Get fast PID correction
        correction = self.cross_track_pid.update(cross_track_error)
        
        # OPTIMIZED: Simplified correction limiting
        max_correction = self.base_speed * VIRTUAL_LINE_MAX_CORRECTION
        correction = max(-max_correction, min(max_correction, correction))
        
        # Apply correction to motor speeds
        left_speed = -self.base_speed + correction   # Left motor (negative = forward)
        right_speed = self.base_speed - correction   # Right motor (positive = forward)
        
        # Send motor commands
        self.motor.send_rpm(1, left_speed)
        self.motor.send_rpm(2, right_speed)
        
        # Check if close enough to target
        distance_to_target = self.get_distance_to_target()
        if distance_to_target < self.target_threshold:
            self.motor.send_rpm(1, 0)
            self.motor.send_rpm(2, 0)
            return True
            
        return False
    
    def check_room_bounds(self, pos: Tuple[float, float]) -> bool:
        """OPTIMIZED: Fast bounds checking"""
        return (0 <= pos[0] <= self.room_width and 0 <= pos[1] <= self.room_height)
    
    def update(self, current_heading: float = None) -> NavigationState:
        """OPTIMIZED: Fast navigation state machine update"""
        current_pos = self.uwb.get_current_position()
        
        # OPTIMIZED: Quick safety check
        if not self.check_room_bounds((current_pos[0], current_pos[1])):
            logger.warning(f"Robot outside bounds: ({current_pos[0]:.2f}, {current_pos[1]:.2f})")
            self.motor.send_rpm(1, 0)
            self.motor.send_rpm(2, 0)
            return self.state
        
        if self.state == NavigationState.IDLE:
            # Waiting for target
            pass
            
        elif self.state == NavigationState.DETECTING_HEADING:
            # OPTIMIZED: Fast heading detection
            if self.detect_heading_by_movement():
                self.state = NavigationState.MOVING_TO_TARGET
                logger.info(f"🚀 Navigation started. Mode: {'Grid' if self.use_grid_mode else 'Virtual Line'}")
                time.sleep(0.3)  # Brief pause
                
        elif self.state == NavigationState.MOVING_TO_TARGET:
            # OPTIMIZED: Main navigation logic
            if self.use_grid_mode:
                # Use fast grid-based movement (fallback)
                if self.grid_based_movement((current_pos[0], current_pos[1])):
                    self.state = NavigationState.REACHED_TARGET
            else:
                # Use fast virtual line following (primary method)
                if self.virtual_line_following((current_pos[0], current_pos[1])):
                    self.state = NavigationState.REACHED_TARGET
            
            # OPTIMIZED: Check if we should switch to magnetic line following
            distance_to_target = self.get_distance_to_target()
            if self.sensor and distance_to_target < self.line_approach_distance:
                # Quick check if magnetic line is detected
                try:
                    _, position_value = self.sensor.read_data()
                    if position_value and position_value != 0xFFFF:
                        logger.info("🧲 Magnetic line detected, switching to line following")
                        self.state = NavigationState.FOLLOWING_LINE
                        self.motor.send_rpm(1, 0)
                        self.motor.send_rpm(2, 0)
                        time.sleep(0.2)
                except:
                    pass  # Continue with UWB navigation if sensor read fails
                
        elif self.state == NavigationState.FOLLOWING_LINE:
            # Line following is handled by the main controller
            distance_to_target = self.get_distance_to_target()
            if distance_to_target < self.target_threshold:
                self.state = NavigationState.REACHED_TARGET
                
        elif self.state == NavigationState.REACHED_TARGET:
            # Target reached, stop
            self.motor.send_rpm(1, 0)
            self.motor.send_rpm(2, 0)
            logger.info("🎯 Navigation complete - target reached successfully!")
            
        return self.state
        
    def stop(self):
        """OPTIMIZED: Emergency stop"""
        self.motor.send_rpm(1, 0)
        self.motor.send_rpm(2, 0)
        self.state = NavigationState.IDLE
        self.cross_track_pid.reset()
        logger.warning("⛔ Navigation stopped")
        
    def get_status(self) -> dict:
        """OPTIMIZED: Get current navigation status with performance info"""
        current_pos = self.uwb.get_current_position()
        
        cross_track_error = 0.0
        if self.state == NavigationState.MOVING_TO_TARGET and not self.use_grid_mode:
            cross_track_error = self.calculate_cross_track_error((current_pos[0], current_pos[1]))
        
        status = {
            'state': self.state.name,
            'current_position': (current_pos[0], current_pos[1]),
            'target_position': self.target_position,
            'distance_to_target': self.get_distance_to_target() if self.target_position else None,
            'robot_heading': self.robot_heading,
            'navigation_mode': 'Grid-Based' if self.use_grid_mode else 'Fast Virtual Line',
            'cross_track_error': cross_track_error,
            'virtual_line_start': self.virtual_line_start,
            'virtual_line_end': self.virtual_line_end,
            'room_bounds': f"{self.room_width}×{self.room_height}m",
            'within_bounds': self.check_room_bounds((current_pos[0], current_pos[1])),
            'optimization': 'Fast Navigation Mode'
        }
        
        return status