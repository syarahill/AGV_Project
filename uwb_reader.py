import serial
import logging
import threading
import time
import math
from typing import Tuple, Optional, Dict
from config import *

logger = logging.getLogger("UWBReader")

class KalmanFilter2D:
    """2D Kalman Filter implementation based on the journal paper equations (23-28)"""
    
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
        
        logger.info(f"Kalman Filter initialized - Process noise: X={process_noise_x}, Y={process_noise_y}, "
                   f"Measurement noise: {measurement_noise}, Initial errors: X={initial_error_x}, Y={initial_error_y}")
    
    def update(self, measurement_x, measurement_y):
        """
        Update Kalman filter with new measurement
        Based on journal equations (23-28)
        """
        if self.first_update:
            # Initialize with first measurement
            self.est_x_prev = measurement_x
            self.est_y_prev = measurement_y
            self.first_update = False
            return measurement_x, measurement_y
        
        # Calculate Kalman Gains (Equations 23-24)
        kg_x = self.error_est_x_prev / (self.error_est_x_prev + self.error_measurement)
        kg_y = self.error_est_y_prev / (self.error_est_y_prev + self.error_measurement)
        
        # Calculate current estimates (Equations 25-26)
        est_x_current = self.est_x_prev + kg_x * (measurement_x - self.est_x_prev)
        est_y_current = self.est_y_prev + kg_y * (measurement_y - self.est_y_prev)
        
        # Update current estimation errors (Equations 27-28)
        error_est_x_current = (1 - kg_x) * self.error_est_x_prev
        error_est_y_current = (1 - kg_y) * self.error_est_y_prev
        
        # Store for next iteration
        self.est_x_prev = est_x_current
        self.est_y_prev = est_y_current
        self.error_est_x_prev = error_est_x_current
        self.error_est_y_prev = error_est_y_current
        
        logger.debug(f"Kalman update - Raw: ({measurement_x:.3f}, {measurement_y:.3f}), "
                    f"Filtered: ({est_x_current:.3f}, {est_y_current:.3f}), "
                    f"Gains: KG_X={kg_x:.3f}, KG_Y={kg_y:.3f}")
        
        return est_x_current, est_y_current
    
    def reset(self):
        """Reset filter to initial state"""
        self.est_x_prev = 0.0
        self.est_y_prev = 0.0
        self.error_est_x_prev = UWB_KALMAN_INITIAL_ERROR_X
        self.error_est_y_prev = UWB_KALMAN_INITIAL_ERROR_Y
        self.first_update = True
        logger.info("Kalman filter reset")

class UWBReader:
    def __init__(self, port: str, baudrate: int = 115200):
        """Initialize UWB reader for HR-RTLS1 system with height compensation"""
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.is_connected = False
        
        # Headers for data parsing
        self.head_raw = ['0x6d', '0x63']  # 'mc' in hex
        self.head_don = ['0x24', '0x4b']  # '$K' in hex
        
        # CORRECTED Anchor positions from config
        anchor_positions = UWB_ANCHOR_POSITIONS
        self.Ax0, self.Ay0 = anchor_positions[0][0], anchor_positions[0][1]  # (4.0, 0.4)
        self.Ax1, self.Ay1 = anchor_positions[1][0], anchor_positions[1][1]  # (4.0, 6.4)
        self.Ax2, self.Ay2 = anchor_positions[2][0], anchor_positions[2][1]  # (0.0, 3.2)
        
        # Current position and distances
        self.posisi_t = (0, 0)
        self.posisi_t_raw = (0, 0)  # Raw trilateration result
        self.posisi_t_filtered = (0, 0)  # Kalman filtered result
        self.anchor_distances = {0: 0.0, 1: 0.0, 2: 0.0}  # Store 3D distances
        self.anchor_distances_2d = {0: 0.0, 1: 0.0, 2: 0.0}  # Store ground distances
        self.distance_timestamps = {0: 0.0, 1: 0.0, 2: 0.0}  # Track distance update times
        self.position_lock = threading.Lock()
        self.running = False
        
        # Initialize Kalman filter if enabled
        self.kalman_filter = None
        if UWB_KALMAN_ENABLED:
            self.kalman_filter = KalmanFilter2D()
            logger.info("Kalman filtering enabled for UWB positioning with height compensation")
        
        # Position validation
        self.last_valid_position = (0, 0)
        self.position_jump_count = 0
        self.trilateration_error_count = 0
        
        # Height compensation parameters
        self.height_difference = UWB_HEIGHT_DIFFERENCE
        logger.info(f"Height compensation enabled: Anchor height={UWB_ANCHOR_HEIGHT}m, "
                   f"AGV height={UWB_AGV_HEIGHT}m, Height difference={self.height_difference}m")
        
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
        """
        Convert 3D distance to 2D ground distance using height compensation
        Formula: ground_distance = sqrt(3d_distance² - height_difference²)
        """
        if distance_3d <= self.height_difference:
            # If 3D distance is less than height difference, something is wrong
            logger.warning(f"3D distance ({distance_3d:.3f}m) <= height difference ({self.height_difference:.3f}m)")
            return 0.1  # Return minimum valid distance
        
        try:
            ground_distance = math.sqrt(distance_3d**2 - self.height_difference**2)
            logger.debug(f"Height compensation: 3D={distance_3d:.3f}m -> Ground={ground_distance:.3f}m")
            return ground_distance
        except ValueError:
            logger.error(f"Height compensation calculation error for distance {distance_3d:.3f}m")
            return distance_3d  # Return original distance if calculation fails
    
    def validate_distances(self, distances: Dict[int, float]) -> bool:
        """Validate distance measurements"""
        current_time = time.time()
        valid_count = 0
        
        for anchor_id, distance in distances.items():
            # Check distance range
            if not (UWB_MIN_DISTANCE <= distance <= UWB_MAX_DISTANCE):
                logger.warning(f"Distance to anchor {anchor_id} out of range: {distance:.3f}m")
                continue
            
            # Update timestamp
            self.distance_timestamps[anchor_id] = current_time
            valid_count += 1
        
        # Check if we have enough recent distances
        recent_distances = 0
        for anchor_id in range(3):
            if current_time - self.distance_timestamps[anchor_id] < UWB_DISTANCE_TIMEOUT:
                recent_distances += 1
        
        return recent_distances >= TRILATERATION_MIN_ANCHORS and valid_count >= TRILATERATION_MIN_ANCHORS
    
    def validate_position(self, x: float, y: float) -> bool:
        """Validate calculated position"""
        # Check bounds (updated for correct room size)
        if not (UWB_MIN_VALID_X <= x <= UWB_MAX_VALID_X and 
                UWB_MIN_VALID_Y <= y <= UWB_MAX_VALID_Y):
            return False
        
        # Check for unrealistic position jumps
        if self.last_valid_position != (0, 0):
            distance_jump = math.sqrt(
                (x - self.last_valid_position[0])**2 + 
                (y - self.last_valid_position[1])**2
            )
            if distance_jump > UWB_MAX_POSITION_JUMP:
                self.position_jump_count += 1
                logger.warning(f"Large position jump detected: {distance_jump:.3f}m (count: {self.position_jump_count})")
                # Allow occasional jumps but not too many
                if self.position_jump_count > 3:
                    return False
        
        return True
    
    def enhanced_trilateration(self, x1, y1, r1, x2, y2, r2, x3, y3, r3):
        """
        Enhanced trilateration using journal paper equations (21-22)
        """
        try:
            # Validate input distances
            if not all(r > 0 for r in [r1, r2, r3]):
                logger.error("Invalid distances for trilateration (negative or zero)")
                return self.posisi_t_raw
            
            # Journal paper equations (21) and (22) - enhanced trilateration
            A = 2 * (x2 - x1)
            B = 2 * (y2 - y1)
            C = r1**2 - r2**2 - x1**2 + x2**2 - y1**2 + y2**2
            D = 2 * (x3 - x2)
            E = 2 * (y3 - y2)
            F = r2**2 - r3**2 - x2**2 + x3**2 - y2**2 + y3**2
            
            # Check for singular matrix (determinant near zero)
            denominator = (E * A - B * D)
            if abs(denominator) < 1e-6:
                logger.warning("Trilateration matrix near singular, using fallback")
                return self.simple_trilateration(x1, y1, r1, x2, y2, r2, x3, y3, r3)
            
            # Calculate position using journal equations
            x = (C * E - F * B) / denominator
            y = (C * D - A * F) / (B * D - A * E)
            
            # Validate the calculated position
            if not self.validate_position(x, y):
                logger.warning(f"Trilateration result failed validation: ({x:.3f}, {y:.3f})")
                self.trilateration_error_count += 1
                
                if self.trilateration_error_count > 10:
                    self.position_jump_count = 0
                    self.trilateration_error_count = 0
                    logger.info("Reset position validation counters")
                
                return self.posisi_t_raw
            
            # Verify solution by checking distances
            calculated_distances = [
                math.sqrt((x - x1)**2 + (y - y1)**2),
                math.sqrt((x - x2)**2 + (y - y2)**2),
                math.sqrt((x - x3)**2 + (y - y3)**2)
            ]
            
            errors = [abs(calc - measured) for calc, measured in zip(calculated_distances, [r1, r2, r3])]
            max_error = max(errors)
            
            if max_error > TRILATERATION_MAX_ERROR:
                logger.warning(f"High trilateration error: {max_error:.3f}m, using fallback")
                return self.simple_trilateration(x1, y1, r1, x2, y2, r2, x3, y3, r3)
            
            # Update last valid position
            self.last_valid_position = (x, y)
            self.position_jump_count = max(0, self.position_jump_count - 1)
            self.trilateration_error_count = 0
            
            logger.debug(f"Enhanced trilateration successful: ({x:.3f}, {y:.3f}), Max error: {max_error:.3f}m")
            
            return x, y
            
        except Exception as e:
            logger.error(f"Enhanced trilateration error: {e}")
            return self.posisi_t_raw
    
    def simple_trilateration(self, x1, y1, r1, x2, y2, r2, x3, y3, r3):
        """Fallback simple trilateration method"""
        try:
            A = 2 * x2 - 2 * x1
            B = 2 * y2 - 2 * y1
            C = r1**2 - r2**2 - x1**2 + x2**2 - y1**2 + y2**2
            D = 2 * x3 - 2 * x2
            E = 2 * y3 - 2 * y2
            F = r2**2 - r3**2 - x2**2 + x3**2 - y2**2 + y3**2
            
            x = (C * E - F * B) / (E * A - B * D)
            y = (C * D - A * F) / (B * D - A * E)
            
            # Basic validation
            if (UWB_MIN_VALID_X <= x <= UWB_MAX_VALID_X and 
                UWB_MIN_VALID_Y <= y <= UWB_MAX_VALID_Y):
                return x, y
            else:
                return self.posisi_t_raw
                
        except Exception as e:
            logger.error(f"Simple trilateration error: {e}")
            return self.posisi_t_raw
    
    def adj_grid(self, x_uwb, y_uwb):
        """Convert UWB position to grid coordinates"""
        if (UWB_MIN_VALID_X <= x_uwb <= UWB_MAX_VALID_X and 
            UWB_MIN_VALID_Y <= y_uwb <= UWB_MAX_VALID_Y):
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
    
    def parse_mc_data(self, data):
        """Parse mc format data from HR-RTLS1"""
        try:
            parts = data.decode('utf-8').strip().split()
            
            if len(parts) >= 10 and parts[0] == 'mc':
                ranges = {}
                for i in range(3):
                    try:
                        range_hex = parts[i + 2]
                        if range_hex != 'ffffffff':
                            range_mm = int(range_hex, 16)
                            ranges[i] = range_mm / 1000.0  # Convert to meters
                    except:
                        pass
                
                return ranges
        except Exception as e:
            logger.error(f"Error parsing mc data: {e}")
        return None
    
    def receive_data(self):
        """Continuously receive and parse UWB data"""
        buffer = b''
        
        while self.running and self.ser and self.ser.isOpen():
            try:
                if self.ser.in_waiting > 0:
                    new_data = self.ser.read(self.ser.in_waiting)
                    buffer += new_data
                    
                    while b'\n' in buffer:
                        line, buffer = buffer.split(b'\n', 1)
                        
                        # Try to parse as mc format first
                        if line.startswith(b'mc'):
                            ranges = self.parse_mc_data(line)
                            if ranges and len(ranges) >= 3:
                                if self.validate_distances(ranges):
                                    self.process_position_update(ranges)
                        
                        # Try to parse as $K format
                        elif line.startswith(b'$K'):
                            try:
                                string_data = line.decode('utf-8')
                                if len(string_data) >= 19:
                                    ranges = {
                                        0: float(string_data[5:9]),
                                        1: float(string_data[10:14]),
                                        2: float(string_data[15:19])
                                    }
                                    
                                    if self.validate_distances(ranges):
                                        self.process_position_update(ranges)
                                        
                            except Exception as e:
                                logger.error(f"Error parsing $K data: {e}")
                
                time.sleep(0.01)
                        
            except Exception as e:
                if self.running:
                    logger.error(f"UWB receive error: {e}")
                time.sleep(0.1)
    
    def process_position_update(self, ranges):
        """Process new distance measurements with height compensation and update position"""
        with self.position_lock:
            # Store original 3D distances
            for anchor_id, distance_3d in ranges.items():
                self.anchor_distances[anchor_id] = distance_3d
                
                # Apply height compensation to get ground distance
                ground_distance = self.apply_height_compensation(distance_3d)
                self.anchor_distances_2d[anchor_id] = ground_distance
            
            # Calculate position using enhanced trilateration with ground distances
            posisi_raw = self.enhanced_trilateration(
                self.Ax0, self.Ay0, self.anchor_distances_2d.get(0, 0),
                self.Ax1, self.Ay1, self.anchor_distances_2d.get(1, 0),
                self.Ax2, self.Ay2, self.anchor_distances_2d.get(2, 0)
            )
            
            self.posisi_t_raw = posisi_raw
            
            # Apply Kalman filtering if enabled
            if self.kalman_filter and UWB_KALMAN_ENABLED:
                posisi_filtered = self.kalman_filter.update(posisi_raw[0], posisi_raw[1])
                self.posisi_t_filtered = posisi_filtered
                self.posisi_t = posisi_filtered
            else:
                self.posisi_t_filtered = posisi_raw
                self.posisi_t = posisi_raw
            
            logger.debug(f"Position update - Raw: ({posisi_raw[0]:.3f}, {posisi_raw[1]:.3f}), "
                        f"Filtered: ({self.posisi_t[0]:.3f}, {self.posisi_t[1]:.3f}), "
                        f"3D Distances: A0={ranges.get(0,0):.2f}m, A1={ranges.get(1,0):.2f}m, A2={ranges.get(2,0):.2f}m, "
                        f"Ground Distances: A0={self.anchor_distances_2d.get(0,0):.2f}m, A1={self.anchor_distances_2d.get(1,0):.2f}m, A2={self.anchor_distances_2d.get(2,0):.2f}m")
    
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
        """Get position quality metrics"""
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
        logger.info("Position filter and validation reset")
    
    def continuous_read(self):
        """Start continuous reading in background"""
        self.running = True
        threading.Thread(target=self.receive_data, daemon=True).start()