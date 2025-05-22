import serial
import struct
import crcmod.predefined
import time
import logging
from config import WHEEL_PORT, WHEEL_BAUDRATE, MOTOR_ID_LEFT, MOTOR_ID_RIGHT

logger = logging.getLogger("MotorControl")

class MotorControl:
    def __init__(self):
        self.ser = serial.Serial(WHEEL_PORT, WHEEL_BAUDRATE, timeout=0.1)
        self.crc8 = crcmod.predefined.mkPredefinedCrcFun("crc-8-maxim")
        self.prev_fb_rpm = [0, 0]
        self.prev_fb_cur = [0, 0]

    def close(self):
        if self.ser.is_open:
            self.ser.close()
            logger.info("Motor serial port closed")

    def send_rpm(self, motor_id, rpm):
        rpm = int(rpm)
        cmd_bytes = struct.pack('>BBBBBBBBB', motor_id, 0x64, (rpm >> 8) & 0xFF, rpm & 0xFF, 0, 0, 0, 0, 0)
        cmd_bytes = self._crc_attach(cmd_bytes)
        self.ser.write(cmd_bytes)

    def set_drive_mode(self, motor_id, mode):
        cmd_bytes = struct.pack('>BBBBBBBBBB', motor_id, 0xA0, 0, 0, 0, 0, 0, 0, 0, mode)
        cmd_bytes = self._crc_attach(cmd_bytes)
        self.ser.write(cmd_bytes)

    def get_feedback(self, motor_id):
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
            return rpm, cur, error
        return self.prev_fb_rpm[motor_id - 1], self.prev_fb_cur[motor_id - 1], 0

    def _crc_attach(self, data_bytes):
        crc_int = self.crc8(data_bytes)
        return data_bytes + bytes([crc_int])

    def _current_raw_to_amp(self, cur_raw):
        return (cur_raw / 32767.0) * 8.0