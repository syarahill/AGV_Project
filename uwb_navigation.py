import math
import time
import logging
import numpy as np
from enum import Enum
from config import *

logger = logging.getLogger("UWBNavigation")

class NavigationState(Enum):
    IDLE = 0
    START = 1
    TURNING = 2
    MOVING = 3
    COMPLETE = 4
    ERROR = 5
    SNAPPING = 6  # New state for grid snapping

class UWBNavigation:
    def __init__(self, motor_control, uwb_reader):
        self.motor_control = motor_control
        self.uwb_reader = uwb_reader
        self.state = NavigationState.IDLE
        self.target_position = None
        self.start_position = None
        self.current_leg = 0
        self.legs = []
        self.action_start_time = 0
        self.desired_rpm = [0, 0]
        self.current_heading = 0.0  # Start facing North (Y+)
        self.last_position = (0, 0)
        self.last_heading_update = time.time()
        
        # Grid navigation variables
        self.current_grid = (0, 0)
        self.target_grid = (0, 0)
        self.initial_heading_set = False
        self.steps_x = 0
        self.steps_y = 0
        self.dir_x = 0
        self.dir_y = 0
        self.remaining_steps = 0
        self.move_direction = 0
        self.move_start_time = 0
        self.move_duration_per_cell = 0.4 / OPEN_LOOP_LINEAR_SPEED  # Time per grid cell
        
        # Wheel physics calculations
        self.turn_time_90 = OPEN_LOOP_TURN_TIME_90
        self.turn_radius = WHEEL_BASE / 2.0  # For pivot turns
        self.rotation_circumference = 2 * math.pi * self.turn_radius
        self.rotation_speed = NAVIGATION_TURN_SPEED * (WHEEL_CIRCUMFERENCE / 60.0)  # m/s
        
        # Navigation completion flag
        self._navigation_complete = False

    def _calculate_turn_time(self, angle):
        """Calculate turn time based on angle and wheel physics"""
        arc_length = abs(angle) * (math.pi / 180) * self.turn_radius
        return arc_length / self.rotation_speed

    def _snap_to_grid(self, position):
        """Snap UWB position to nearest grid center"""
        x, y = position
        grid_x = round((x - 0.2) / CELL_SIZE)  # Cell centers at 0.2, 0.6, etc.
        grid_y = round((y - 0.2) / CELL_SIZE)
        snapped_x = (grid_x * CELL_SIZE) + 0.2
        snapped_y = (grid_y * CELL_SIZE) + 0.2
        return (grid_x, grid_y), (snapped_x, snapped_y)

    def navigate_to_position(self, x: float, y: float):
        """Start navigation to target position using grid-based movement"""
        if not (0 <= x <= ROOM_WIDTH and 0 <= y <= ROOM_HEIGHT):
            logger.error(f"Target ({x}, {y}) outside room bounds")
            return False
        
        # Reset completion flag
        self._navigation_complete = False
        
        # Get initial position and snap to grid
        init_pos = self.uwb_reader.get_current_position()[:2]
        self.current_grid, snapped_pos = self._snap_to_grid(init_pos)
        self.start_position = snapped_pos
        self.last_position = snapped_pos
        
        logger.info(f"Starting from grid {self.current_grid} (pos: {init_pos[0]:.2f}, {init_pos[1]:.2f})")
        
        # Convert target to grid coordinates
        self.target_grid, _ = self._snap_to_grid((x, y))
        logger.info(f"Target grid: {self.target_grid} (pos: {x:.2f}, {y:.2f})")
        
        # Calculate grid steps
        dx = self.target_grid[0] - self.current_grid[0]
        dy = self.target_grid[1] - self.current_grid[1]
        self.steps_x = abs(dx)
        self.steps_y = abs(dy)
        self.dir_x = 1 if dx > 0 else -1
        self.dir_y = 1 if dy > 0 else -1
        
        # Set initial heading (0° = North/+Y)
        if not self.initial_heading_set:
            self.current_heading = 0
            self.initial_heading_set = True
        
        # Plan X then Y movement
        self.legs = []
        if self.steps_x > 0:
            self.legs.append((self.steps_x, 90 if self.dir_x > 0 else 270))  # X movement
        if self.steps_y > 0:
            self.legs.append((self.steps_y, 0 if self.dir_y > 0 else 180))    # Y movement
        
        if not self.legs:
            logger.info("Already at target position!")
            self._navigation_complete = True
            self.state = NavigationState.COMPLETE
            return True
        
        self.state = NavigationState.START
        self.current_leg = 0
        logger.info(f"Navigation plan: {len(self.legs)} legs, X steps: {self.steps_x}, Y steps: {self.steps_y}")
        return True

    def _start_leg(self, current_time):
        """Start a movement leg"""
        if self.current_leg >= len(self.legs):
            logger.error("No more legs to execute!")
            self.state = NavigationState.ERROR
            return
            
        steps, target_heading = self.legs[self.current_leg]
        
        # Calculate turn angle
        turn_angle = (target_heading - self.current_heading) % 360
        if turn_angle > 180:
            turn_angle -= 360  # Prefer shorter turn
        
        # Calculate turn time based on physics
        turn_time = self._calculate_turn_time(turn_angle)
        
        if abs(turn_angle) < ROTATION_THRESHOLD:
            # Already facing correct direction
            self._start_moving(current_time, steps)
        else:
            # Determine turn direction
            if turn_angle > 0:
                self.desired_rpm = [NAVIGATION_TURN_SPEED, NAVIGATION_TURN_SPEED]  # Right turn
            else:
                self.desired_rpm = [-NAVIGATION_TURN_SPEED, -NAVIGATION_TURN_SPEED]  # Left turn
                
            self.state = NavigationState.TURNING
            self.action_start_time = current_time
            self.turn_duration = abs(turn_time)
            self.target_heading_after_turn = target_heading
            logger.info(f"[NAV] Turning {abs(turn_angle):.1f}° from {self.current_heading}° to {target_heading}° (duration: {self.turn_duration:.2f}s)")

    def _start_moving(self, current_time, steps):
        """Start moving forward for given steps"""
        self.remaining_steps = steps
        self.move_direction = self.legs[self.current_leg][1]
        self.state = NavigationState.MOVING
        self.move_start_time = current_time
        self.desired_rpm = [NAVIGATION_MOVE_SPEED, -NAVIGATION_MOVE_SPEED]  # Forward
        
        # Calculate move duration for grid cell
        move_direction_name = "X" if self.move_direction in [90, 270] else "Y"
        total_duration = steps * self.move_duration_per_cell
        logger.info(f"[NAV] Moving {move_direction_name} for {steps} cells (duration: {total_duration:.2f}s)")

    def _update_heading(self, current_time, current_x, current_y):
        """Discrete heading update - no continuous recalculation"""
        # We only update heading when turning, not during movement
        return

    def update(self, current_time):
        """Update navigation state machine for grid movement"""
        # Return current state for debugging
        if self.state == NavigationState.COMPLETE:
            return True  # Navigation is complete
            
        if self.state == NavigationState.START:
            self._start_leg(current_time)
            return False  # Not complete yet
            
        elif self.state == NavigationState.TURNING:
            if current_time - self.action_start_time >= self.turn_duration:
                # Turn complete
                logger.info(f"[NAV] Turn complete")
                self.motor_control.emergency_stop()
                time.sleep(0.05)  # Brief pause
                self.current_heading = self.target_heading_after_turn
                steps = self.legs[self.current_leg][0]
                self._start_moving(current_time, steps)
            return False  # Not complete yet
                
        elif self.state == NavigationState.MOVING:
            elapsed = current_time - self.move_start_time
            cell_progress = elapsed / self.move_duration_per_cell
            
            if cell_progress >= 1.0:
                # Completed one cell
                self.remaining_steps -= 1
                
                # Update grid position
                if self.move_direction == 0:   # North
                    self.current_grid = (self.current_grid[0], self.current_grid[1] + 1)
                elif self.move_direction == 90:  # East
                    self.current_grid = (self.current_grid[0] + 1, self.current_grid[1])
                elif self.move_direction == 180:  # South
                    self.current_grid = (self.current_grid[0], self.current_grid[1] - 1)
                elif self.move_direction == 270:  # West
                    self.current_grid = (self.current_grid[0] - 1, self.current_grid[1])
                
                # Update real position (grid center)
                self.last_position = (
                    (self.current_grid[0] * CELL_SIZE) + 0.2,
                    (self.current_grid[1] * CELL_SIZE) + 0.2
                )
                
                if self.remaining_steps > 0:
                    # Start next cell
                    self.move_start_time = current_time
                    logger.debug(f"[NAV] Moved to cell {self.current_grid}, {self.remaining_steps} steps remaining")
                else:
                    # Leg complete
                    logger.info(f"[NAV] Leg {self.current_leg + 1} complete")
                    self.motor_control.emergency_stop()
                    time.sleep(0.05)  # Brief pause
                    self.current_leg += 1
                    
                    if self.current_leg < len(self.legs):
                        # Start next leg
                        logger.info(f"[NAV] Starting leg {self.current_leg + 1} of {len(self.legs)}")
                        self.state = NavigationState.START
                    else:
                        # Navigation complete
                        logger.info(f"[NAV] All legs complete! Final position: grid {self.current_grid}")
                        self.state = NavigationState.COMPLETE
                        self._navigation_complete = True
                        return True  # Navigation is complete
            return False  # Not complete yet
            
        elif self.state == NavigationState.ERROR:
            logger.error("Navigation in ERROR state")
            return False
            
        return False  # Default: not complete

    def is_navigation_complete(self):
        """Check if navigation is complete"""
        return self._navigation_complete or self.state == NavigationState.COMPLETE

    def get_current_position(self) -> tuple:
        """Get current grid-based position"""
        return (*self.last_position, 0.0)

    def get_current_grid(self) -> tuple:
        """Get current grid coordinates"""
        return self.current_grid

    def apply_motor_commands(self):
        """Apply calculated motor commands"""
        if self.state in [NavigationState.TURNING, NavigationState.MOVING]:
            self.motor_control.send_rpm(MOTOR_ID_LEFT, self.desired_rpm[0])
            self.motor_control.send_rpm(MOTOR_ID_RIGHT, self.desired_rpm[1])

    def reset(self):
        """Reset navigation state"""
        logger.info("[NAV] Resetting navigation state")
        self.state = NavigationState.IDLE
        self.target_position = None
        self.start_position = None
        self.current_leg = 0
        self.legs = []
        self.action_start_time = 0
        self.desired_rpm = [0, 0]
        self.current_heading = 0.0
        self.last_position = (0, 0)
        self.last_heading_update = time.time()
        self.current_grid = (0, 0)
        self.remaining_steps = 0
        self._navigation_complete = False