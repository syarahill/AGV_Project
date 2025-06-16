import enum
import math

# Serial ports
WHEEL_PORT = "COM3"  # Motor controller port
SENSOR_PORT = "COM15"  # Magnetic sensor port
UWB_PORT = "COM12"  # UWB module port 

# Baudrates
WHEEL_BAUDRATE = 115200  # Baudrate for motor controller
SENSOR_BAUDRATE = 9600  # Baudrate for sensor
UWB_BAUDRATE = 115200  # Baudrate for UWB module

# Motor parameters
MOTOR_ID_LEFT = 1
MOTOR_ID_RIGHT = 2
MAX_SPEED = 20  # RPM
TURN_SPEED = 10  # RPM for turning

# Sensor parameters
SENSOR_ADDRESS = 1  # Modbus address for sensor

# Corrected sensor configuration based on actual setup
# Sensor mounted on front, 16 points with 10mm spacing
LOW_SENSORS = [1, 2, 3, 4, 5, 6, 7, 8]     # RIGHT side sensors (low numbers)
HIGH_SENSORS = [9, 10, 11, 12, 13, 14, 15, 16]  # LEFT side sensors (high numbers)
CENTER_SENSORS = [7, 8, 9, 10]              # Center sensors for ideal position
INNER_SENSORS = [6, 7, 8, 9, 10, 11]        # Inner sensors for fine detection

# Physical layout: [1][2][3][4][5][6][7][8] | [9][10][11][12][13][14][15][16]
#                      RIGHT SIDE           |           LEFT SIDE

# PID constants for line following - Your optimized values
INITIAL_PID_KP = 2.0      # Restored from old code
INITIAL_PID_KI = 0.0      # Restored from old code
INITIAL_PID_KD = 0.5      # Restored from old code

# Line following parameters
SETPOINT = 8.5            # Center point between sensors 8 and 9
IDEAL_CENTER = 8.5        # Ideal center position
MAX_CORRECTION = 15       # Maximum correction speed
MIN_CORRECTION = 2        # Minimum correction to apply

# RESTORED FROM OLD CODE: Original timing for better line following
LINE_LOST_TIMEOUT = 0.5   
MOVING_AVERAGE_WINDOW = 3 
SENSOR_OFFSET = 0.15      # meters
MOTOR_UPDATE_RATE = 50    # RESTORED: Back to 50ms from 100ms
SENSOR_READ_RATE = 25     # RESTORED: Back to 25ms from 50ms
UWB_UPDATE_RATE = 30      # Keep optimized UWB rate

# Detection thresholds
MIN_SENSORS_FOR_LINE = 1  # Minimum sensors needed to detect line
MAX_SENSORS_FOR_LINE = 6  # Maximum sensors (if more, might be noise)

# CORRECTED ROOM AND UWB CONFIGURATION
# Room Specifications: 11 columns × 17 rows, 40×40cm cells
ROOM_WIDTH = 4.4          # meters (11 columns × 0.4m)
ROOM_HEIGHT = 6.8         # meters (17 rows × 0.4m)
GRID_SIZE_X = 11          # columns
GRID_SIZE_Y = 17          # rows
CELL_SIZE = 0.4           # meters per cell (40cm)

# UWB Anchor Positions (CORRECTED)
UWB_ANCHOR_POSITIONS = {
    0: (4.0, 6.4, 2.5),   # A0 at grid (10,1)
    1: (4.0, 0.4, 2.5),   # A1 at grid (10,16)
    2: (0.0, 3.2, 2.5),   # A2 at grid (0,8)
}

# UWB Height Compensation
UWB_ANCHOR_HEIGHT = 2.5   # meters - height of anchors above ground
UWB_AGV_HEIGHT = 0.2      # meters - AGV operates on ground level
UWB_HEIGHT_DIFFERENCE = UWB_ANCHOR_HEIGHT - UWB_AGV_HEIGHT  # 2.5m

# Navigation parameters (UPDATED for better performance)
TARGET_THRESHOLD = 0.1    # meters - how close to target before stopping (reduced from 0.15)
ROTATION_THRESHOLD = 15.0  # degrees - rotation accuracy (increased from 3.0)
LINE_APPROACH_DISTANCE = 0.5  # meters - when to switch to line following

# Movement speeds for navigation
NAVIGATION_MOVE_SPEED = 15  # RPM for forward movement during navigation (increased from 12)
NAVIGATION_TURN_SPEED = 10  # RPM for rotation during navigation (increased from 8)

# UWB Kalman Filter Parameters
UWB_KALMAN_ENABLED = True
UWB_KALMAN_DT = 0.05       # Time step in seconds
UWB_KALMAN_PROCESS_NOISE = 0.1  # Process noise (variance of acceleration)
UWB_KALMAN_MEASUREMENT_NOISE = 0.05  # Measurement noise (variance of UWB measurements)

# OPTIMIZED UWB Position Validation Parameters (simplified for speed)
UWB_MAX_VALID_X = ROOM_WIDTH         # Maximum valid X coordinate (4.4m)
UWB_MAX_VALID_Y = ROOM_HEIGHT        # Maximum valid Y coordinate (6.8m)
UWB_MIN_VALID_X = 0.0                # Minimum valid X coordinate
UWB_MIN_VALID_Y = 0.0                # Minimum valid Y coordinate
UWB_MAX_POSITION_JUMP = 5.0          # OPTIMIZED: Increased tolerance from 2.0 to 5.0

# OPTIMIZED UWB Distance Validation (simplified)
UWB_MIN_DISTANCE = 0.1     # Minimum valid distance to anchor (meters)
UWB_MAX_DISTANCE = 25.0    # Maximum valid distance to anchor (meters)
UWB_DISTANCE_TIMEOUT = 2.0 # OPTIMIZED: Increased timeout for better stability

# Trilateration Parameters (simplified)
TRILATERATION_MIN_ANCHORS = 3        # Minimum anchors needed for trilateration
TRILATERATION_MAX_ERROR = 10.0       # OPTIMIZED: Increased tolerance from 5.0

# NEW: Grid Navigation Parameters
GRID_MOVEMENT_SPEED = 15     # RPM for grid movement
GRID_TURN_SPEED = 10         # RPM for turning
GRID_POSITION_THRESHOLD = 0.25  # Grid position accuracy (meters)
GRID_TURN_TIMEOUT = 5.0      # Maximum time for a turn (seconds)
GRID_MOVE_TIMEOUT = 10.0     # Maximum time for a move (seconds)

# NEW: Open-loop navigation parameters
OPEN_LOOP_LINEAR_SPEED = 0.2  # m/s, measured forward speed at NAVIGATION_MOVE_SPEED RPM
OPEN_LOOP_TURN_TIME_90 = 2.0  # seconds to turn 90 degrees at NAVIGATION_TURN_SPEED RPM

# NEW: Task 1 Parameters  
TASK1_TARGET_GRID = (6, 10)  # Target grid position for Task 1

# Heading Detection Parameters
HEADING_DETECTION_MIN_DISTANCE = 0.5  # Increased from 0.3 for better accuracy
HEADING_DETECTION_TIMEOUT = 3.0       # Increased from 2.0 for better accuracy
HEADING_DETECTION_SPEED = 12          # Speed for heading detection movement

# Grid Movement Parameters (Fallback)
GRID_MOVEMENT_THRESHOLD = 0.2         # Distance threshold for grid-based movement

# PERFORMANCE OPTIMIZATION: GUI Update Parameters
GUI_POSITION_UPDATE_THRESHOLD = 0.05  # Only update GUI if AGV moved 5cm
GUI_HEAVY_UPDATE_INTERVAL = 3         # Update heavy operations every 3rd cycle
GUI_UPDATE_RATE = 50                  # Base GUI update rate in ms

# PERFORMANCE OPTIMIZATION: UWB Processing
UWB_DATA_RATE_LIMIT = 0.02            # Minimum interval between position updates (50Hz max)
UWB_BUFFER_SIZE = 1024                # Maximum buffer size for serial data

# Empirical bias correction parameters for UWB X-axis
UWB_X_BIAS = -0.1   # Constant offset to apply to X coordinate
UWB_X_SCALE = 1.0   # Scale factor to apply to X coordinate

# Grid navigation parameters
GRID_MOVEMENT_MODE = "MANHATTAN"  # X-then-Y movement
GRID_HEADING_MODE = "DISCRETE"  # Only 0/90/180/270 degree headings
GRID_SNAP_THRESHOLD = 0.2  # Max drift before re-snap to grid

# Wheel physics parameters
WHEEL_BASE = 0.26  # Distance between wheel centers (meters)
WHEEL_DIAMETER = 0.1  # Wheel diameter (meters)
WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER  # 0.314m

# Enums for AGV states
class AGVState(enum.Enum):
    DISCONNECTED = 0
    IDLE = 1
    MANUAL_DRIVE = 2
    LINE_FOLLOW = 3
    ERROR = 4
    LINE_LOST = 5
    UWB_NAVIGATION = 6  # Added for UWB navigation
    TASK1_NAVIGATION = 7  # NEW: Added for Task 1