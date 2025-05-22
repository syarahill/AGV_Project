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
MIDDLE_SENSORS = [7, 8, 9, 10]  # Sensors considered for line detection

# PID constants for line following
INITIAL_PID_KP = 4.2
INITIAL_PID_KI = 0.008
INITIAL_PID_KD = 4.0
SETPOINT = 8.5  # Center of sensors 7, 8, 9, 10

# Timing and thresholds
LINE_LOST_TIMEOUT = 0.3  # seconds
MOVING_AVERAGE_WINDOW = 2
MIN_CORRECTION = 2.5
SENSOR_OFFSET = 0.15  # meters
MOTOR_UPDATE_RATE = 25  # ms
SENSOR_READ_RATE = 50  # ms

# Enums for AGV states
class AGVState(enum.Enum):
    DISCONNECTED = 0
    IDLE = 1
    MANUAL_DRIVE = 2
    LINE_FOLLOW = 3
    ERROR = 4