import serial
import logging
import threading
import time
import math
import numpy as np
from scipy.optimize import least_squares
from typing import Tuple, Optional, Dict
from config import *

logger = logging.getLogger("UWBReader")

class KalmanFilter2D:
    """2D constant-velocity Kalman filter for UWB-based positioning"""
    
    def __init__(self, dt=UWB_KALMAN_DT, 
                 var_acc=UWB_KALMAN_PROCESS_NOISE,
                 var_meas=UWB_KALMAN_MEASUREMENT_NOISE):
        # Time step
        self.dt = dt
        # State vector [x, y, vx, vy]
        self.x = np.zeros(4)
        # Covariance matrix
        self.P = np.eye(4) * 100.0
        
        # State transition matrix (constant velocity model)
        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
        # Process noise covariance
        q = var_acc
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt2 * dt2
        self.Q = q * np.array([
            [dt4/4, 0, dt3/2, 0],
            [0, dt4/4, 0, dt3/2],
            [dt3/2, 0, dt2, 0],
            [0, dt3/2, 0, dt2]
        ])
        
        # Measurement matrix
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])
        
        # Measurement noise covariance
        self.R = np.eye(2) * var_meas

    def predict(self):
        """Predict next state (time update)"""
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, meas: Tuple[float, float]) -> Tuple[float, float]:
        """Update with a new measurement"""
        z = np.array(meas)
        # Innovation covariance
        S = self.H @ self.P @ self.H.T + self.R
        # Kalman gain
        K = self.P @ self.H.T @ np.linalg.inv(S)
        # State update
        y = z - self.H @ self.x
        self.x = self.x + K @ y
        # Covariance update
        self.P = (np.eye(4) - K @ self.H) @ self.P
        return float(self.x[0]), float(self.x[1])


class UWBReader:
    """Optimized UWB positioning with nonlinear least-squares and Kalman filtering"""
    
    def __init__(self, port: str, baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.is_connected = False
        
        # Anchor positions
        self.anchors = np.array([
            UWB_ANCHOR_POSITIONS[0][:2],
            UWB_ANCHOR_POSITIONS[1][:2],
            UWB_ANCHOR_POSITIONS[2][:2]
        ])
        
        # Current state
        self.position_raw = (0.0, 0.0)  # Raw trilaterated position
        self.position = (0.0, 0.0, 0.0)  # Final position (with Kalman)
        self.anchor_distances = {0: 0.0, 1: 0.0, 2: 0.0}  # 3D distances
        self.anchor_distances_2d = {0: 0.0, 1: 0.0, 2: 0.0}  # Ground distances
        
        # Kalman filter
        self.kf = KalmanFilter2D() if UWB_KALMAN_ENABLED else None
        
        # Threading
        self.position_lock = threading.Lock()
        self.running = False
        self.last_position_update = 0

    def connect(self) -> bool:
        """Connect to UWB module"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.1,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            self.is_connected = True
            logger.info(f"Connected to UWB on {self.port}")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to UWB: {e}")
            self.is_connected = False
            return False

    def close(self):
        """Close UWB connection"""
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.is_connected = False
            logger.info("UWB connection closed")

    def apply_height_compensation(self, distance_3d: float) -> float:
        """Convert 3D distance to ground distance"""
        if distance_3d <= UWB_HEIGHT_DIFFERENCE:
            return 0.1  # Minimum valid distance
        return math.sqrt(max(distance_3d**2 - UWB_HEIGHT_DIFFERENCE**2, 0))

    def trilaterate(self, distances: Dict[int, float]) -> Tuple[float, float]:
        """Nonlinear least-squares trilateration with height compensation"""
        # Convert to arrays
        anchor_pos = self.anchors
        dist_values = np.array([
            self.apply_height_compensation(distances.get(0, 0)),
            self.apply_height_compensation(distances.get(1, 0)),
            self.apply_height_compensation(distances.get(2, 0))
        ])
        
        # Initial guess: centroid of anchors
        x0 = np.mean(anchor_pos, axis=0)
        
        def residuals(p):
            """Compute distance residuals for current position estimate"""
            dx = p[0] - anchor_pos[:, 0]
            dy = p[1] - anchor_pos[:, 1]
            return np.sqrt(dx*dx + dy*dy) - dist_values
        
        # Solve nonlinear least-squares
        result = least_squares(residuals, x0, method='lm')
        x_est, y_est = result.x
        
        # Apply empirical bias correction for X-axis
        x_corrected = (x_est + UWB_X_BIAS) * UWB_X_SCALE
        
        # Clamp to room bounds
        x_corrected = max(0.0, min(ROOM_WIDTH, x_corrected))
        y_est = max(0.0, min(ROOM_HEIGHT, y_est))
        
        return float(x_corrected), float(y_est)

    def parse_mc_data(self, data: bytes) -> Optional[Dict[int, float]]:
        """Parse UWB distance data"""
        try:
            parts = data.decode('utf-8', errors='ignore').strip().split()
            if len(parts) < 5 or parts[0] != 'mc':
                return None
                
            ranges = {}
            for i in range(3):  # For 3 anchors
                hex_str = parts[i+2]
                if hex_str != 'ffffffff':
                    ranges[i] = int(hex_str, 16) / 1000.0  # Convert mm to meters
            return ranges if len(ranges) >= 3 else None
        except Exception as e:
            logger.error(f"Data parsing error: {e}")
            return None

    def receive_data(self):
        """Continuous data reception thread"""
        buffer = b''
        while self.running and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    # Read available data
                    new_data = self.ser.read(min(self.ser.in_waiting, UWB_BUFFER_SIZE))
                    buffer += new_data
                    
                    # Process complete lines
                    lines = buffer.split(b'\n')
                    buffer = lines[-1]  # Keep incomplete line
                    
                    for line in lines[:-1]:
                        if line.startswith(b'mc') and len(line) > 20:
                            ranges = self.parse_mc_data(line)
                            if ranges:
                                self.update_position(ranges)
                
                time.sleep(0.01)
            except Exception as e:
                logger.error(f"UWB receive error: {e}")
                time.sleep(0.1)

    def update_position(self, ranges: Dict[int, float]):
        """Update position based on new range measurements"""
        current_time = time.time()
        
        # Rate limiting
        if current_time - self.last_position_update < UWB_DATA_RATE_LIMIT:
            return
            
        self.last_position_update = current_time
        
        with self.position_lock:
            # Update distances
            for anchor_id, distance in ranges.items():
                self.anchor_distances[anchor_id] = distance
                self.anchor_distances_2d[anchor_id] = self.apply_height_compensation(distance)
            
            # Perform trilateration
            try:
                pos_raw = self.trilaterate(self.anchor_distances)
                self.position_raw = pos_raw
                
                # Apply Kalman filtering if enabled
                if self.kf:
                    self.kf.predict()
                    x_f, y_f = self.kf.update(pos_raw)
                    self.position = (x_f, y_f, 0.0)
                else:
                    self.position = (pos_raw[0], pos_raw[1], 0.0)
                    
            except Exception as e:
                logger.error(f"Trilateration error: {e}")

    def get_current_position(self) -> Tuple[float, float, float]:
        """Get current filtered position"""
        with self.position_lock:
            return self.position

    def get_position_quality(self) -> Dict:
        """Get positioning diagnostics"""
        with self.position_lock:
            return {
                'raw_position': self.position_raw,
                'filtered_position': self.position[:2],
                'anchor_distances_3d': self.anchor_distances.copy(),
                'anchor_distances_2d': self.anchor_distances_2d.copy(),
                'height_difference': UWB_HEIGHT_DIFFERENCE,
                'kalman_enabled': UWB_KALMAN_ENABLED
            }

    def reset_position_filter(self):
        """Reset Kalman filter"""
        if self.kf:
            self.kf = KalmanFilter2D()
            logger.info("Kalman filter reset")

    def continuous_read(self):
        """Start continuous reading in background thread"""
        self.running = True
        threading.Thread(target=self.receive_data, daemon=True).start()