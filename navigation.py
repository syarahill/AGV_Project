import math
import logging
import time
from typing import Tuple, List, Optional
from enum import Enum

# Import config items individually to avoid import issues
try:
    from config import (
        GRID_SIZE_X, GRID_SIZE_Y, CELL_SIZE, ROOM_WIDTH, ROOM_HEIGHT,
        GRID_MOVEMENT_SPEED, GRID_TURN_SPEED, GRID_POSITION_THRESHOLD,
        GRID_TURN_TIMEOUT, GRID_MOVE_TIMEOUT, TASK1_TARGET_GRID
    )
except ImportError:
    # Fallback values if config import fails
    GRID_SIZE_X = 11
    GRID_SIZE_Y = 17
    CELL_SIZE = 0.4
    ROOM_WIDTH = 4.4
    ROOM_HEIGHT = 6.8
    GRID_MOVEMENT_SPEED = 15
    GRID_TURN_SPEED = 10
    GRID_POSITION_THRESHOLD = 0.25
    GRID_TURN_TIMEOUT = 3.0
    GRID_MOVE_TIMEOUT = 10.0
    TASK1_TARGET_GRID = (6, 10)

logger = logging.getLogger("PathNavigation")

class NavigationState(Enum):
    IDLE = 0
    PLANNING_PATH = 1
    EXECUTING_STEP = 2
    TURNING = 3
    MOVING_FORWARD = 4
    STEP_COMPLETE = 5
    REACHED_TARGET = 6
    VALIDATING_ARRIVAL = 7

class PathStep:
    def __init__(self, action: str, direction: int = None, from_grid: Tuple[int, int] = None, to_grid: Tuple[int, int] = None):
        self.action = action  # "turn" or "move"
        self.direction = direction  # 0=North, 90=East, 180=South, 270=West
        self.from_grid = from_grid
        self.to_grid = to_grid
        self.completed = False

class PathNavigation:
    def __init__(self, uwb_reader, motor_control):
        """Simple path-based navigation with visualization"""
        self.uwb = uwb_reader
        self.motor = motor_control
        
        # Navigation state
        self.target_grid = None
        self.current_heading = 0  # Always start facing North (Y+)
        self.state = NavigationState.IDLE
        
        # Path planning
        self.planned_path = []  # List of PathStep objects
        self.current_step_index = 0
        self.current_step = None
        
        # Movement parameters
        self.move_speed = GRID_MOVEMENT_SPEED
        self.turn_speed = GRID_TURN_SPEED
        
        # Step execution
        self.step_start_time = 0
        self.step_start_position = None
        self.step_start_grid = None
        
        # Timing
        self.turn_duration = 1.0  # Time to turn 90 degrees
        self.move_duration = 3.0  # Time to move one grid cell
        
        # Final validation
        self.arrival_check_time = 0
        self.validation_duration = 2.0  # Time to wait for UWB to stabilize
        
        logger.info("Path Navigation initialized - Always starts facing North (Y+)")
        
    def real_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert real coordinates to grid coordinates"""
        grid_x = int(round(x / CELL_SIZE))
        grid_y = int(round(y / CELL_SIZE))
        # Clamp to valid range
        grid_x = max(0, min(GRID_SIZE_X - 1, grid_x))
        grid_y = max(0, min(GRID_SIZE_Y - 1, grid_y))
        return grid_x, grid_y
    
    def grid_to_real(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """Convert grid coordinates to real coordinates (center of cell)"""
        real_x = (grid_x + 0.5) * CELL_SIZE
        real_y = (grid_y + 0.5) * CELL_SIZE
        return real_x, real_y
    
    def calculate_path(self, start_grid: Tuple[int, int], target_grid: Tuple[int, int]) -> List[PathStep]:
        """Calculate the complete path from start to target"""
        path = []
        current_x, current_y = start_grid
        target_x, target_y = target_grid
        current_heading = 0  # Always start facing North
        
        logger.info(f"Planning path from {start_grid} to {target_grid}")
        
        # Move X-axis first (East/West), then Y-axis (North/South)
        
        # Step 1: Move X-axis if needed
        if current_x != target_x:
            required_heading = 90 if target_x > current_x else 270  # East or West
            
            # Add turn if needed
            if current_heading != required_heading:
                path.append(PathStep("turn", required_heading))
                current_heading = required_heading
            
            # Add moves for X-axis
            steps_x = abs(target_x - current_x)
            for i in range(steps_x):
                from_grid = (current_x, current_y)
                current_x += 1 if target_x > current_x else -1
                to_grid = (current_x, current_y)
                path.append(PathStep("move", current_heading, from_grid, to_grid))
        
        # Step 2: Move Y-axis if needed
        if current_y != target_y:
            required_heading = 0 if target_y > current_y else 180  # North or South
            
            # Add turn if needed
            if current_heading != required_heading:
                path.append(PathStep("turn", required_heading))
                current_heading = required_heading
            
            # Add moves for Y-axis
            steps_y = abs(target_y - current_y)
            for i in range(steps_y):
                from_grid = (current_x, current_y)
                current_y += 1 if target_y > current_y else -1
                to_grid = (current_x, current_y)
                path.append(PathStep("move", current_heading, from_grid, to_grid))
        
        # Log the planned path
        logger.info(f"Planned path with {len(path)} steps:")
        for i, step in enumerate(path):
            if step.action == "turn":
                logger.info(f"  {i+1}. Turn to {step.direction}°")
            else:
                logger.info(f"  {i+1}. Move from {step.from_grid} to {step.to_grid}")
        
        return path
    
    def get_path_visualization(self) -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:
        """Get path as list of line segments for GUI visualization"""
        if not self.planned_path:
            return []
        
        lines = []
        
        for step in self.planned_path:
            if step.action == "move" and step.from_grid and step.to_grid:
                from_real = self.grid_to_real(step.from_grid[0], step.from_grid[1])
                to_real = self.grid_to_real(step.to_grid[0], step.to_grid[1])
                lines.append((from_real, to_real))
        
        return lines
    
    def set_target_grid(self, grid_x: int, grid_y: int) -> bool:
        """Set target and plan the path"""
        if not (0 <= grid_x < GRID_SIZE_X and 0 <= grid_y < GRID_SIZE_Y):
            logger.error(f"Target grid ({grid_x}, {grid_y}) is outside bounds")
            return False
        
        # Get current position from UWB
        try:
            current_pos = self.uwb.get_current_position()
            current_grid = self.real_to_grid(current_pos[0], current_pos[1])
        except Exception as e:
            logger.error(f"Failed to get current position: {e}")
            return False
        
        self.target_grid = (grid_x, grid_y)
        self.current_heading = 0  # Always start facing North
        
        # Calculate the complete path
        self.planned_path = self.calculate_path(current_grid, self.target_grid)
        self.current_step_index = 0
        self.current_step = None
        
        if self.planned_path:
            self.state = NavigationState.EXECUTING_STEP
            logger.info(f"Path planned: {len(self.planned_path)} steps from {current_grid} to {self.target_grid}")
        else:
            # Already at target
            self.state = NavigationState.REACHED_TARGET
            logger.info(f"Already at target {self.target_grid}")
        
        return True
    
    def start_turn(self, target_heading: int):
        """Start turning to target heading"""
        self.step_start_time = time.time()
        
        # Calculate turn direction (shortest path)
        heading_diff = (target_heading - self.current_heading) % 360
        if heading_diff > 180:
            heading_diff -= 360
        
        if abs(heading_diff) < 5:  # Already facing correct direction
            self.current_heading = target_heading
            self.state = NavigationState.STEP_COMPLETE
            return
        
        try:
            if heading_diff > 0:
                # Turn right (clockwise)
                self.motor.send_rpm(1, -self.turn_speed)  # Left motor reverse
                self.motor.send_rpm(2, -self.turn_speed)  # Right motor reverse
                logger.debug(f"Turning RIGHT from {self.current_heading}° to {target_heading}°")
            else:
                # Turn left (counter-clockwise)
                self.motor.send_rpm(1, self.turn_speed)   # Left motor forward
                self.motor.send_rpm(2, self.turn_speed)   # Right motor forward
                logger.debug(f"Turning LEFT from {self.current_heading}° to {target_heading}°")
            
            self.state = NavigationState.TURNING
        except Exception as e:
            logger.error(f"Error starting turn: {e}")
            self.state = NavigationState.STEP_COMPLETE
    
    def start_move_forward(self):
        """Start moving forward one grid cell"""
        self.step_start_time = time.time()
        
        try:
            current_pos = self.uwb.get_current_position()
            self.step_start_position = current_pos
            self.step_start_grid = self.real_to_grid(current_pos[0], current_pos[1])
            
            # Move forward (robot always uses same motor commands for forward)
            self.motor.send_rpm(1, -self.move_speed)  # Left motor reverse (forward)
            self.motor.send_rpm(2, self.move_speed)   # Right motor forward
            
            logger.debug(f"Moving forward (heading: {self.current_heading}°) from grid {self.step_start_grid}")
            self.state = NavigationState.MOVING_FORWARD
        except Exception as e:
            logger.error(f"Error starting move: {e}")
            self.state = NavigationState.STEP_COMPLETE
    
    def stop_motors(self):
        """Stop all motor movement"""
        try:
            self.motor.send_rpm(1, 0)
            self.motor.send_rpm(2, 0)
        except Exception as e:
            logger.error(f"Error stopping motors: {e}")
    
    def check_step_completion(self) -> bool:
        """Check if current step is completed"""
        current_time = time.time()
        
        if self.state == NavigationState.TURNING:
            # Check if turn duration elapsed
            if current_time - self.step_start_time >= self.turn_duration:
                self.stop_motors()
                if self.current_step:
                    self.current_heading = self.current_step.direction
                logger.debug(f"Turn completed, now facing {self.current_heading}°")
                return True
                
        elif self.state == NavigationState.MOVING_FORWARD:
            # Check if move duration elapsed
            if current_time - self.step_start_time >= self.move_duration:
                self.stop_motors()
                logger.debug(f"Move completed (time-based)")
                return True
            
            # Also check if we've moved to next grid cell
            try:
                current_pos = self.uwb.get_current_position()
                current_grid = self.real_to_grid(current_pos[0], current_pos[1])
                if current_grid != self.step_start_grid:
                    self.stop_motors()
                    logger.debug(f"Move completed (grid changed): {self.step_start_grid} -> {current_grid}")
                    return True
            except Exception as e:
                logger.debug(f"Error checking position during move: {e}")
        
        return False
    
    def update(self) -> NavigationState:
        """Update navigation state machine"""
        if self.state == NavigationState.IDLE:
            return self.state
        
        elif self.state == NavigationState.EXECUTING_STEP:
            # Check if all steps are completed
            if self.current_step_index >= len(self.planned_path):
                self.state = NavigationState.VALIDATING_ARRIVAL
                self.arrival_check_time = time.time()
                logger.info("All path steps completed, validating arrival...")
                return self.state
            
            # Get next step
            self.current_step = self.planned_path[self.current_step_index]
            
            if self.current_step.action == "turn":
                self.start_turn(self.current_step.direction)
            elif self.current_step.action == "move":
                self.start_move_forward()
        
        elif self.state in [NavigationState.TURNING, NavigationState.MOVING_FORWARD]:
            # Check if current step is complete
            if self.check_step_completion():
                if self.current_step:
                    self.current_step.completed = True
                self.current_step_index += 1
                self.state = NavigationState.EXECUTING_STEP
                time.sleep(0.2)  # Brief pause between steps
        
        elif self.state == NavigationState.VALIDATING_ARRIVAL:
            # Wait for UWB to stabilize, then check final position
            if time.time() - self.arrival_check_time >= self.validation_duration:
                try:
                    current_pos = self.uwb.get_current_position()
                    current_grid = self.real_to_grid(current_pos[0], current_pos[1])
                    
                    if current_grid == self.target_grid:
                        self.state = NavigationState.REACHED_TARGET
                        logger.info(f"🎯 Successfully reached target grid {self.target_grid}!")
                    else:
                        logger.warning(f"Position validation failed: at {current_grid}, target was {self.target_grid}")
                        # For now just mark as reached
                        self.state = NavigationState.REACHED_TARGET
                except Exception as e:
                    logger.error(f"Error during arrival validation: {e}")
                    self.state = NavigationState.REACHED_TARGET
        
        elif self.state == NavigationState.REACHED_TARGET:
            # Stay stopped at target
            self.stop_motors()
        
        return self.state
    
    def stop(self):
        """Emergency stop navigation"""
        self.stop_motors()
        self.state = NavigationState.IDLE
        self.target_grid = None
        self.planned_path = []
        logger.warning("Navigation stopped")
    
    def get_status(self) -> dict:
        """Get current navigation status"""
        try:
            current_pos = self.uwb.get_current_position()
            current_grid = self.real_to_grid(current_pos[0], current_pos[1])
            
            # Calculate progress
            total_steps = len(self.planned_path)
            completed_steps = self.current_step_index
            progress = (completed_steps / total_steps * 100) if total_steps > 0 else 100
            
            # Current step info
            current_step_info = "None"
            if self.current_step:
                if self.current_step.action == "turn":
                    current_step_info = f"Turn to {self.current_step.direction}°"
                else:
                    current_step_info = f"Move {self.current_step.from_grid} -> {self.current_step.to_grid}"
            
            return {
                'state': self.state.name,
                'current_grid': current_grid,
                'current_position': (current_pos[0], current_pos[1]),
                'target_grid': self.target_grid,
                'current_heading': self.current_heading,
                'progress': f"{completed_steps}/{total_steps} ({progress:.0f}%)",
                'current_step': current_step_info,
                'path_visualization': self.get_path_visualization(),
                'total_steps': total_steps,
                'completed_steps': completed_steps
            }
        except Exception as e:
            logger.error(f"Error getting status: {e}")
            return {
                'state': self.state.name,
                'current_grid': (0, 0),
                'current_position': (0.0, 0.0),
                'target_grid': self.target_grid,
                'current_heading': self.current_heading,
                'progress': "Error",
                'current_step': "Error",
                'path_visualization': [],
                'total_steps': 0,
                'completed_steps': 0
            }