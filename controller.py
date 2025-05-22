import threading
import queue
import time
import logging
from collections import deque
from config import *
from sensor_reader import SensorReader
from motor_control import MotorControl

logger = logging.getLogger("Controller")

class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.last_error) / dt if dt > 0 else 0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.last_error = error
        return output

class Controller:
    def __init__(self):
        self.sensor = SensorReader()
        self.motor_control = MotorControl()
        self.command_queue = queue.Queue()
        self.sensor_queue = queue.Queue()
        self.state = AGVState.DISCONNECTED
        self.pid = PID(INITIAL_PID_KP, INITIAL_PID_KI, INITIAL_PID_KD)
        self.desired_rpm = [0, 0]
        self.pid_error = 0.0
        self.lock = threading.Lock()
        self.running = False

    def start(self):
        self.running = True
        threading.Thread(target=self._sensor_loop, daemon=True).start()
        threading.Thread(target=self._control_loop, daemon=True).start()

    def stop(self):
        self.running = False
        self.sensor.close()
        self.motor_control.close()

    def _sensor_loop(self):
        if not self.sensor.connect():
            self.state = AGVState.ERROR
            return
        while self.running:
            data = self.sensor.read_data()
            if data:
                self.sensor_queue.put(data)
            time.sleep(SENSOR_READ_RATE / 1000.0)

    def _control_loop(self):
        while self.running:
            try:
                cmd = self.command_queue.get_nowait()
                if cmd[0] == "set_state":
                    self.state = cmd[1]
                elif cmd[0] == "set_speed":
                    self.desired_rpm = cmd[1]
                elif cmd[0] == "update_pid":
                    self.pid.kp, self.pid.ki, self.pid.kd = cmd[1]
            except queue.Empty:
                pass

            if self.state == AGVState.LINE_FOLLOW:
                try:
                    median_value, position_value = self.sensor_queue.get_nowait()
                    active_bits = [i + 1 for i in range(16) if position_value & (1 << i)]
                    if active_bits:
                        location = sum(active_bits) / len(active_bits)
                        error = SETPOINT - location
                        dt = time.time() - (self.pid.last_time or time.time())
                        correction = self.pid.compute(error, dt)
                        self.desired_rpm = [MAX_SPEED + correction, - (MAX_SPEED - correction)]
                        self.pid_error = error
                except queue.Empty:
                    pass

            self.motor_control.send_rpm(MOTOR_ID_LEFT, self.desired_rpm[0])
            self.motor_control.send_rpm(MOTOR_ID_RIGHT, self.desired_rpm[1])
            time.sleep(MOTOR_UPDATE_RATE / 1000.0)