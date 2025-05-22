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

    def connect(self):
        try:
            self.is_connected = self.client.connect()
            if self.is_connected:
                logger.info(f"Connected to sensor on {SENSOR_PORT}")
            return self.is_connected
        except Exception as e:
            logger.error(f"Failed to connect to sensor: {e}")
            return False

    def read_data(self):
        if not self.is_connected:
            return None, None
        try:
            result = self.client.read_holding_registers(address=0, count=2, slave=self.address)
            if result.isError():
                logger.error(f"Modbus error: {result}")
                return None, None
            registers = result.registers
            median_value = registers[0] / 10.0  # Assuming scaling
            position_value = registers[1]
            return median_value, position_value
        except Exception as e:
            logger.error(f"Sensor read error: {e}")
            return None, None

    def close(self):
        if self.is_connected:
            self.client.close()
            self.is_connected = False
            logger.info("Sensor connection closed")