import serial
import logging
import threading
import time
import math
from typing import Tuple, Optional, Dict
from config import *

logger = logging.getLogger("UWBReader")

class KalmanFilter2D:
    """Optimized 2D Kalman Filter implementation based on the journal paper equations (23-28)"""
    
    def __init__(self, process_noise_x=UWB_KALMAN_PROCESS_NOISE_X, 
                 process_noise_y=UWB_KALMAN_PROCESS_NOISE_Y,
                 measurement_noise=UWB_KALMAN_MEASUREMENT_NOISE,
                 initial_error_x=UWB_KALMAN_INITIAL_ERROR_X,
                 initial_error_y=UWB_KALMAN_INITIAL_ERROR_Y):
        
        # Initial estimates
        self.est_x_prev = 0.0  # EST_X(t-1)
        self.est_y_prev = 0.0  # EST_Y(t-1)
        
        # Initial errors
        self.error_est_x_prev = initial_error_x  # E_EST_X(t-1)
        self.error_est_y_prev = initial_error_y  # E_EST_Y(t-1)
        
        # Noise parameters
        self.error_measurement = measurement_noise  # E_MEA
        
        # First update flag
        self.first_update = True
        
        logger.info(f"Optimized Kalman Filter initialized - Measurement noise: {measurement_noise}")
    
    def update(self, measurement_x, measurement_y):
        """OPTIMIZED: Fast Kalman filter update with minimal calculations"""
        if self.first_update:
            self.est_x_prev = measurement_x
            self.est_y_prev = measurement_y
            self.first_update = False
            return measurement_x, measurement_y
        
        # Fast Kalman Gains calculation
        kg_x = self.error_est_x_prev / (self.error_est_x_prev + self.error_measurement)
        kg_y = self.error_est_y_prev / (self.error_est_y_prev + self.error_measurement)
        
        # Fast estimates calculation
        est_x_current = self.est_x_prev + kg_x * (measurement_x - self.est_x_prev)
        est_y_current = self.est_y_prev + kg_y * (measurement_y - self.est_y_prev)
        
        # Fast error updates
        self.error_est_x_prev = (1 - kg_x) * self.error_est_x_prev
        self.error_est_y_prev = (1 - kg_y) * self.error_est_y_prev
        
        # Store for next iteration
        self.est_x_prev = est_x_current
        self.est_y_prev = est_y_current
        
        return est_x_current, est_y_current
    
    def reset(self):
        """Reset filter to initial state"""
        self.est_x_prev = 0.0
        self.est_y_prev = 0.0
        self.error_est_x_prev = UWB_KALMAN_INITIAL_ERROR_X
        self.error_est_y_prev = UWB_KALMAN_INITIAL_ERROR_Y
        self.first_update = True

class UWBReader:
    def __init__(self, port: str, baudrate: int = 115200):
        """OPTIMIZED: Initialize UWB reader with fast processing"""
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.is_connected = False
        
        # OPTIMIZED: Use anchor positions directly from config
        anchor_positions = UWB_ANCHOR_POSITIONS
        self.Ax0, self.Ay0 = anchor_positions[0][0], anchor_positions[0][1]
        self.Ax1, self.Ay1 = anchor_positions[1][0], anchor_positions[1][1]
        self.Ax2, self.Ay2 = anchor_positions[2][0], anchor_positions[2][1]
        
        # Current position and distances
        self.posisi_t = (0, 0)
        self.posisi_t_raw = (0, 0)
        self.posisi_t_filtered = (0, 0)
        self.anchor_distances = {0: 0.0, 1: 0.0, 2: 0.0}
        self.anchor_distances_2d = {0: 0.0, 1: 0.0, 2: 0.0}
        self.position_lock = threading.Lock()
        self.running = False
        
        # OPTIMIZED: Simplified Kalman filter
        self.kalman_filter = None
        if UWB_KALMAN_ENABLED:
            self.kalman_filter = KalmanFilter2D()
        
        # OPTIMIZED: Minimal validation tracking
        self.last_valid_position = (0, 0)
        self.position_jump_count = 0
        self.trilateration_error_count = 0
        
        # Height compensation
        self.height_difference = UWB_HEIGHT_DIFFERENCE
        
        # PERFORMANCE: Rate limiting
        self.last_position_update = 0
        self.min_update_interval = UWB_DATA_RATE_LIMIT
        
        logger.info(f"Optimized UWB Reader initialized for {ROOM_WIDTH}×{ROOM_HEIGHT}m room")
        
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
            self.running = True
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
        """OPTIMIZED: Fast height compensation calculation"""
        if distance_3d <= self.height_difference:
            return 0.1  # Minimum valid distance
        
        try:
            return math.sqrt(distance_3d**2 - self.height_difference**2)
        except:
            return distance_3d
    
    def validate_distances_fast(self, distances: Dict[int, float]) -> bool:
        """OPTIMIZED: Fast distance validation"""
        return (len(distances) >= 3 and 
                all(UWB_MIN_DISTANCE <= d <= UWB_MAX_DISTANCE for d in distances.values()))
    
    def fast_trilateration(self, x1, y1, r1, x2, y2, r2, x3, y3, r3):
        """OPTIMIZED: Fast trilateration with minimal validation"""
        try:
            # Quick sanity check only
            if not all(0.1 <= r <= 25.0 for r in [r1, r2, r3]):
                return self.posisi_t_raw
            
            # Simple, fast calculation
            A = 2 * (x2 - x1)
            B = 2 * (y2 - y1)
            C = r1**2 - r2**2 - x1**2 + x2**2 - y1**2 + y2**2
            D = 2 * (x3 - x2)
            E = 2 * (y3 - y2)
            F = r2**2 - r3**2 - x2**2 + x3**2 - y2**2 + y3**2
            
            denominator = (E * A - B * D)
            if abs(denominator) < 1e-6:
                return self.posisi_t_raw
            
            x = (C * E - F * B) / denominator
            y = (A * F - C * D) / denominator
            
            # Quick bounds check only
            if 0 <= x <= ROOM_WIDTH and 0 <= y <= ROOM_HEIGHT:
                return x, y
            else:
                return self.posisi_t_raw
                
        except:
            return self.posisi_t_raw
    
    def adj_grid(self, x_uwb, y_uwb):
        """Convert UWB position to grid coordinates"""
        if (0 <= x_uwb <= ROOM_WIDTH and 0 <= y_uwb <= ROOM_HEIGHT):
            x = int(x_uwb // CELL_SIZE)
            y = int(y_uwb // CELL_SIZE)
            return x, y
        else:
            return None, None
    
    def thres_goal(self, x_adj, y_adj):
        """Calculate threshold goal for grid position"""
        x_thres = (x_adj + 0.5) * CELL_SIZE
        y_thres = (y_adj + 0.5) * CELL_SIZE
        return x_thres, y_thres
    
    def parse_mc_data_fast(self, data):
        """OPTIMIZED: Faster MC data parsing"""
        try:
            parts = data.decode('utf-8', errors='ignore').strip().split()
            
            if len(parts) >= 5 and parts[0] == 'mc':
                ranges = {}
                for i in range(3):
                    if i + 2 < len(parts):
                        range_hex = parts[i + 2]
                        if range_hex != 'ffffffff':
                            try:
                                ranges[i] = int(range_hex, 16) / 1000.0
                            except ValueError:
                                continue
                return ranges if len(ranges) >= 3 else None
        except:
            pass
        return None
    
    def receive_data(self):
        """OPTIMIZED: Fast data reception with rate limiting"""
        buffer = b''
        
        while self.running and self.ser and self.ser.isOpen():
            try:
                if self.ser.in_waiting > 0:
                    # OPTIMIZED: Limit read size for better performance
                    new_data = self.ser.read(min(self.ser.in_waiting, UWB_BUFFER_SIZE))
                    buffer += new_data
                    
                    # OPTIMIZED: Process multiple lines at once
                    lines = buffer.split(b'\n')
                    buffer = lines[-1]  # Keep incomplete line
                    
                    for line in lines[:-1]:
                        if line.startswith(b'mc') and len(line) > 20:
                            ranges = self.parse_mc_data_fast(line)
                            if ranges and len(ranges) >= 3:
                                self.process_position_update(ranges)
                                break  # Process only first valid line per cycle
                
                time.sleep(0.02)  # OPTIMIZED: Slightly slower but more stable
                        
            except Exception as e:
                if self.running:
                    logger.error(f"UWB receive error: {e}")
                time.sleep(0.1)
    
    def process_position_update(self, ranges):
        """OPTIMIZED: Fast position update with rate limiting"""
        current_time = time.time()
        
        # PERFORMANCE: Rate limiting to prevent excessive updates
        if current_time - self.last_position_update < self.min_update_interval:
            return
        
        self.last_position_update = current_time
        
        with self.position_lock:
            # OPTIMIZED: Store distances quickly
            for anchor_id, distance_3d in ranges.items():
                self.anchor_distances[anchor_id] = distance_3d
                self.anchor_distances_2d[anchor_id] = self.apply_height_compensation(distance_3d)
            
            # OPTIMIZED: Fast trilateration - NO complex validation
            posisi_raw = self.fast_trilateration(
                self.Ax0, self.Ay0, self.anchor_distances_2d.get(0, 0),
                self.Ax1, self.Ay1, self.anchor_distances_2d.get(1, 0),
                self.Ax2, self.Ay2, self.anchor_distances_2d.get(2, 0)
            )
            
            self.posisi_t_raw = posisi_raw
            
            # OPTIMIZED: Apply Kalman filtering if enabled
            if self.kalman_filter and UWB_KALMAN_ENABLED:
                self.posisi_t_filtered = self.kalman_filter.update(posisi_raw[0], posisi_raw[1])
                self.posisi_t = self.posisi_t_filtered
            else:
                self.posisi_t = posisi_raw
    
    def get_anchor_distances(self) -> Dict[int, float]:
        """Get current 3D distances to all anchors"""
        with self.position_lock:
            return self.anchor_distances.copy()
    
    def get_anchor_distances_2d(self) -> Dict[int, float]:
        """Get current ground distances to all anchors"""
        with self.position_lock:
            return self.anchor_distances_2d.copy()
    
    def read_position(self) -> Optional[Tuple[float, float, float]]:
        """Read current position from UWB"""
        with self.position_lock:
            x, y = self.posisi_t
            return (x, y, 0.0)
    
    def get_current_position(self) -> Tuple[float, float, float]:
        """Get the last known position"""
        with self.position_lock:
            x, y = self.posisi_t
            return (x, y, 0.0)
    
    def get_position_quality(self) -> Dict:
        """OPTIMIZED: Simplified position quality metrics"""
        with self.position_lock:
            return {
                'raw_position': self.posisi_t_raw,
                'filtered_position': self.posisi_t_filtered,
                'position_jump_count': self.position_jump_count,
                'trilateration_error_count': self.trilateration_error_count,
                'kalman_enabled': UWB_KALMAN_ENABLED and self.kalman_filter is not None,
                'anchor_distances_3d': self.anchor_distances.copy(),
                'anchor_distances_2d': self.anchor_distances_2d.copy(),
                'height_compensation': self.height_difference
            }
    
    def reset_position_filter(self):
        """Reset Kalman filter and position validation"""
        if self.kalman_filter:
            self.kalman_filter.reset()
        self.position_jump_count = 0
        self.trilateration_error_count = 0
        logger.info("Position filter reset")
    
    def continuous_read(self):
        """Start continuous reading in background"""
        self.running = True
        threading.Thread(target=self.receive_data, daemon=True).start()