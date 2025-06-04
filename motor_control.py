import serial
import struct
import crcmod.predefined
import time
import logging
from config import WHEEL_PORT, WHEEL_BAUDRATE, MOTOR_ID_LEFT, MOTOR_ID_RIGHT

logger = logging.getLogger("MotorControl")

class MotorControl:
    def __init__(self):
        self.ser = None
        self.crc8 = crcmod.predefined.mkPredefinedCrcFun("crc-8-maxim")
        self.prev_fb_rpm = [0, 0]
        self.prev_fb_cur = [0, 0]
        self.connect()

    def connect(self):
        """Connect to motor controller"""
        try:
            self.ser = serial.Serial(WHEEL_PORT, WHEEL_BAUDRATE, timeout=0.1)
            logger.info(f"Connected to motor controller on {WHEEL_PORT}")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to motor controller: {e}")
            return False

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            logger.info("Motor serial port closed")

    def send_rpm(self, motor_id, rpm):
        """Send RPM command to motor"""
        if not self.ser or not self.ser.is_open:
            logger.error("Motor serial port not open")
            return False
            
        try:
            rpm = int(rpm)
            # Clamp RPM to reasonable limits
            rpm = max(-50, min(50, rpm))
            
            cmd_bytes = struct.pack('>BBBBBBBBB', motor_id, 0x64, (rpm >> 8) & 0xFF, rpm & 0xFF, 0, 0, 0, 0, 0)
            cmd_bytes = self._crc_attach(cmd_bytes)
            self.ser.write(cmd_bytes)
            
            logger.debug(f"Motor {motor_id}: RPM command {rpm}")
            return True
        except Exception as e:
            logger.error(f"Error sending RPM to motor {motor_id}: {e}")
            return False

    def set_drive_mode(self, motor_id, mode):
        """Set drive mode for motor"""
        if not self.ser or not self.ser.is_open:
            logger.error("Motor serial port not open")
            return False
            
        try:
            cmd_bytes = struct.pack('>BBBBBBBBBB', motor_id, 0xA0, 0, 0, 0, 0, 0, 0, 0, mode)
            cmd_bytes = self._crc_attach(cmd_bytes)
            self.ser.write(cmd_bytes)
            logger.info(f"Motor {motor_id}: Drive mode set to {mode}")
            return True
        except Exception as e:
            logger.error(f"Error setting drive mode for motor {motor_id}: {e}")
            return False

    def get_feedback(self, motor_id):
        """Get feedback from motor"""
        if not self.ser or not self.ser.is_open:
            return self.prev_fb_rpm[motor_id - 1], self.prev_fb_cur[motor_id - 1], 0
            
        try:
            cmd_bytes = struct.pack('>BBBBBBBBB', motor_id, 0x74, 0, 0, 0, 0, 0, 0, 0)
            cmd_bytes = self._crc_attach(cmd_bytes)
            self.ser.write(cmd_bytes)
            
            response = self.ser.read(11)
            if len(response) == 11:
                cur_raw = struct.unpack('>h', response[2:4])[0]
                rpm = struct.unpack('>h', response[4:6])[0]
                error = response[8]
                cur = self._current_raw_to_amp(cur_raw)
                
                self.prev_fb_rpm[motor_id - 1] = rpm
                self.prev_fb_cur[motor_id - 1] = cur
                
                logger.debug(f"Motor {motor_id} feedback: RPM={rpm}, Current={cur:.2f}A, Error={error}")
                return rpm, cur, error
        except Exception as e:
            logger.error(f"Error getting feedback from motor {motor_id}: {e}")
            
        return self.prev_fb_rpm[motor_id - 1], self.prev_fb_cur[motor_id - 1], 0

    def _crc_attach(self, data_bytes):
        """Attach CRC to data"""
        crc_int = self.crc8(data_bytes)
        return data_bytes + bytes([crc_int])

    def _current_raw_to_amp(self, cur_raw):
        """Convert raw current value to amperes"""
        return (cur_raw / 32767.0) * 8.0

    def initialize_motors(self):
        """Initialize both motors with proper drive mode"""
        logger.info("Initializing motors...")
        success = True
        
        # Set drive mode for both motors (assuming mode 1 is velocity control)
        success &= self.set_drive_mode(MOTOR_ID_LEFT, 1)
        success &= self.set_drive_mode(MOTOR_ID_RIGHT, 1)
        
        # Stop both motors initially
        success &= self.send_rpm(MOTOR_ID_LEFT, 0)
        success &= self.send_rpm(MOTOR_ID_RIGHT, 0)
        
        if success:
            logger.info("Motors initialized successfully")
        else:
            logger.error("Motor initialization failed")
            
        return success

    def emergency_stop(self):
        """Emergency stop both motors"""
        logger.warning("Emergency stop activated!")
        self.send_rpm(MOTOR_ID_LEFT, 0)
        self.send_rpm(MOTOR_ID_RIGHT, 0)