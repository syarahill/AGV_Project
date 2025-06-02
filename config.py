import enum

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
INITIAL_PID_KP = 2.0      # Your preferred value
INITIAL_PID_KI = 0.0      # Your preferred value
INITIAL_PID_KD = 0.5      # Your preferred value

# Line following parameters
SETPOINT = 8.5            # Center point between sensors 8 and 9
IDEAL_CENTER = 8.5        # Ideal center position
MAX_CORRECTION = 15       # Maximum correction speed
MIN_CORRECTION = 2        # Minimum correction to apply

# OPTIMIZED Timing and thresholds for better performance
LINE_LOST_TIMEOUT = 0.5   
MOVING_AVERAGE_WINDOW = 3 
SENSOR_OFFSET = 0.15      # meters
MOTOR_UPDATE_RATE = 100   # OPTIMIZED: Increased from 50ms to 100ms for smoother control
SENSOR_READ_RATE = 50     # OPTIMIZED: Increased from 25ms to 50ms for better performance
UWB_UPDATE_RATE = 30      # OPTIMIZED: Reduced from 50ms to 30ms for faster updates

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
    0: (4.0, 0.4, 2.5),   # A0 at grid (10,1)
    1: (4.0, 6.4, 2.5),   # A1 at grid (10,16)
    2: (0.0, 3.2, 2.5),   # A2 at grid (0,8)
}

# UWB Height Compensation
UWB_ANCHOR_HEIGHT = 2.5   # meters - height of anchors above ground
UWB_AGV_HEIGHT = 0.0      # meters - AGV operates on ground level
UWB_HEIGHT_DIFFERENCE = UWB_ANCHOR_HEIGHT - UWB_AGV_HEIGHT  # 2.5m

# Navigation parameters (UPDATED for correct room size)
TARGET_THRESHOLD = 0.15   # meters - how close to target before stopping
ROTATION_THRESHOLD = 3.0  # degrees - rotation accuracy
LINE_APPROACH_DISTANCE = 0.5  # meters - when to switch to line following

# Movement speeds for navigation
NAVIGATION_MOVE_SPEED = 12  # RPM for forward movement during navigation
NAVIGATION_TURN_SPEED = 8   # RPM for rotation during navigation

# OPTIMIZED UWB Kalman Filter Parameters (simplified for performance)
UWB_KALMAN_ENABLED = True
UWB_KALMAN_PROCESS_NOISE_X = 0.1     # Process noise for X axis (E_EST)
UWB_KALMAN_PROCESS_NOISE_Y = 0.1     # Process noise for Y axis (E_EST)
UWB_KALMAN_MEASUREMENT_NOISE = 0.2   # OPTIMIZED: Reduced from 0.33 for faster convergence
UWB_KALMAN_INITIAL_ERROR_X = 1.0     # Initial estimation error X
UWB_KALMAN_INITIAL_ERROR_Y = 1.0     # Initial estimation error Y

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
GRID_TURN_TIMEOUT = 3.0      # Maximum time for a turn (seconds)
GRID_MOVE_TIMEOUT = 10.0     # Maximum time for a move (seconds)

# NEW: Task 1 Parameters  
TASK1_TARGET_GRID = (6, 10)  # Target grid position for Task 1

# Heading Detection Parameters
HEADING_DETECTION_MIN_DISTANCE = 0.3  # Minimum movement to detect heading (meters)
HEADING_DETECTION_TIMEOUT = 2.0       # Timeout for heading detection (seconds)
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