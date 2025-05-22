import enum

# Serial ports
WHEEL_PORT = "COM4"  # Motor controller port
SENSOR_PORT = "COM12"  # Magnetic sensor port
WHEEL_BAUDRATE = 115200  # Baudrate for motor controller
SENSOR_BAUDRATE = 9600  # Baudrate for sensor

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

# PID constants for line following - Improved values
INITIAL_PID_KP = 3.0      # Reduced for smoother response
INITIAL_PID_KI = 0.1      # Increased for better steady-state
INITIAL_PID_KD = 2.0      # Reduced to prevent oscillation

# Line following parameters
SETPOINT = 8.5            # Center point between sensors 8 and 9
IDEAL_CENTER = 8.5        # Ideal center position
MAX_CORRECTION = 15       # Maximum correction speed
MIN_CORRECTION = 2        # Minimum correction to apply

# Timing and thresholds
LINE_LOST_TIMEOUT = 0.5   # Increased timeout
MOVING_AVERAGE_WINDOW = 3 # Increased for smoother readings
SENSOR_OFFSET = 0.15      # meters
MOTOR_UPDATE_RATE = 50    # Increased to 50ms for smoother control
SENSOR_READ_RATE = 25     # Faster sensor reading at 25ms

# Detection thresholds
MIN_SENSORS_FOR_LINE = 1  # Minimum sensors needed to detect line
MAX_SENSORS_FOR_LINE = 6  # Maximum sensors (if more, might be noise)

# Enums for AGV states
class AGVState(enum.Enum):
    DISCONNECTED = 0
    IDLE = 1
    MANUAL_DRIVE = 2
    LINE_FOLLOW = 3
    ERROR = 4
    LINE_LOST = 5  # New state for when line is lost