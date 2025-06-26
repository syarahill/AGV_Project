import serial
import threading
import time
import logging
from config import BATTERY_PORT, BATTERY_BAUDRATE, BATTERY_VOLTAGE_JUMP

logger = logging.getLogger("BatteryChecker")

class BatteryChecker:
    def __init__(self, port, baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.voltage = 0.0
        self.percentage = 0
        self.status = "UNKNOWN"
        self.running = False
        self.thread = None
        self.last_voltage = 0.0
        self.charging_detected = False
        # Add charging detection variables
        self.charging_start_voltage = 0.0
        self.charging_start_time = 0.0

    def connect(self):
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            logger.info(f"Connected to battery monitor on {self.port}")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to battery monitor: {e}")
            return False

    def start(self):
        if not self.connect():
            return
            
        self.running = True
        self.thread = threading.Thread(target=self.read_loop)
        self.thread.daemon = True
        self.thread.start()
        logger.info("Battery monitoring started")

    def stop(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        if self.ser and self.ser.is_open:
            self.ser.close()
        logger.info("Battery monitoring stopped")

    def parse_battery_data(self, line):
        """Parse battery data from serial line"""
        if not line.startswith("BAT:"):
            return None
            
        try:
            parts = line.split(':')
            if len(parts) >= 4:
                voltage = float(parts[1])
                percentage = int(parts[2])
                status = parts[3].strip()
                return voltage, percentage, status
        except Exception as e:
            logger.error(f"Error parsing battery data: {e}")
        return None

    def read_loop(self):
        buffer = ""
        while self.running and self.ser and self.ser.is_open:
            try:
                # Read all available data
                data = self.ser.read(self.ser.in_waiting or 1)
                if data:
                    decoded = data.decode('utf-8', errors='ignore')
                    buffer += decoded
                    
                    # Process complete lines
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        if line:
                            result = self.parse_battery_data(line)
                            if result:
                                self.voltage, self.percentage, self.status = result
                                
                                # Detect charging start
                                if self.last_voltage > 0:
                                    voltage_jump = self.voltage - self.last_voltage
                                    if voltage_jump >= BATTERY_VOLTAGE_JUMP:
                                        self.charging_detected = True
                                        self.charging_start_voltage = self.voltage
                                        self.charging_start_time = time.time()
                                        logger.info(f"Charging detected! Voltage jump: {voltage_jump:.2f}V")
                                
                                self.last_voltage = self.voltage
                
                time.sleep(0.1)
            except Exception as e:
                logger.error(f"Battery read error: {e}")
                time.sleep(1)
                # Try to reconnect
                try:
                    self.ser.close()
                except:
                    pass
                self.connect()
    
    def is_charging_confirmed(self, min_increase=0.02):
        """Check if voltage has increased by at least min_increase since charging start"""
        if not self.charging_detected:
            return False
            
        # Calculate voltage increase since charging start
        voltage_increase = self.voltage - self.charging_start_voltage
        return voltage_increase >= min_increase

    def get_data(self):
        return (self.voltage, self.percentage, self.status)