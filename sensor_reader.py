import logging
from pymodbus.client import ModbusSerialClient
from config import SENSOR_PORT, SENSOR_BAUDRATE, SENSOR_ADDRESS

logger = logging.getLogger("SensorReader")

class SensorReader:
    def __init__(self):
        self.client = ModbusSerialClient(
            port=SENSOR_PORT,
            baudrate=SENSOR_BAUDRATE,
            parity='N',
            stopbits=1,
            bytesize=8,
            timeout=0.2
        )
        self.address = SENSOR_ADDRESS
        self.is_connected = False
        self.consecutive_errors = 0
        self.max_consecutive_errors = 5

    def connect(self):
        """Connect to the magnetic sensor"""
        try:
            self.is_connected = self.client.connect()
            if self.is_connected:
                logger.info(f"Connected to sensor on {SENSOR_PORT}")
                self.consecutive_errors = 0
            else:
                logger.error(f"Failed to connect to sensor on {SENSOR_PORT}")
            return self.is_connected
        except Exception as e:
            logger.error(f"Failed to connect to sensor: {e}")
            self.is_connected = False
            return False

    def read_data(self):
        """Read sensor data via Modbus"""
        if not self.is_connected:
            logger.debug("Sensor not connected, attempting to reconnect...")
            if not self.connect():
                return None, None
        
        try:
            result = self.client.read_holding_registers(address=0, count=2, slave=self.address)
            
            if result.isError():
                self.consecutive_errors += 1
                logger.error(f"Modbus error: {result}")
                
                # If too many consecutive errors, try to reconnect
                if self.consecutive_errors >= self.max_consecutive_errors:
                    logger.warning("Too many consecutive errors, attempting reconnect...")
                    self.is_connected = False
                    self.consecutive_errors = 0
                    
                return None, None
            
            # Successful read
            self.consecutive_errors = 0
            registers = result.registers
            
            # Process the data
            median_value = registers[0] / 10.0  # Assuming scaling factor of 10
            position_value = registers[1]
            
            logger.debug(f"Sensor data - Median: {median_value}, Position: 0x{position_value:04X}")
            
            # Validate data (basic sanity checks)
            if median_value < 0 or median_value > 1000:  # Adjust range as needed
                logger.warning(f"Unusual median value: {median_value}")
            
            return median_value, position_value
            
        except Exception as e:
            self.consecutive_errors += 1
            logger.error(f"Sensor read error: {e}")
            
            # If too many consecutive errors, mark as disconnected
            if self.consecutive_errors >= self.max_consecutive_errors:
                logger.warning("Too many consecutive errors, marking sensor as disconnected")
                self.is_connected = False
                self.consecutive_errors = 0
                
            return None, None

    def close(self):
        """Close sensor connection"""
        if self.is_connected:
            try:
                self.client.close()
                self.is_connected = False
                logger.info("Sensor connection closed")
            except Exception as e:
                logger.error(f"Error closing sensor connection: {e}")

    def test_connection(self):
        """Test if sensor is responding"""
        if not self.is_connected:
            return False
            
        try:
            result = self.client.read_holding_registers(address=0, count=1, slave=self.address)
            return not result.isError()
        except Exception as e:
            logger.error(f"Connection test failed: {e}")
            return False