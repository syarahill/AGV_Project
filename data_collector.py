# data_collector.py
import time
import threading
import csv
import json
from collections import deque
from datetime import datetime
import logging

logger = logging.getLogger("DataCollector")

class DataCollector:
    def __init__(self, max_points=1000):
        self.max_points = max_points
        self.lock = threading.Lock()
        
        # Data storage
        self.timestamps = deque(maxlen=max_points)
        self.line_positions = deque(maxlen=max_points)
        self.pid_errors = deque(maxlen=max_points)
        self.pid_outputs = deque(maxlen=max_points)
        self.left_motor_speeds = deque(maxlen=max_points)
        self.right_motor_speeds = deque(maxlen=max_points)
        self.active_sensors = deque(maxlen=max_points)
        self.setpoints = deque(maxlen=max_points)
        
        # Recording state
        self.is_recording = False
        self.start_time = None
        self.session_name = None
        
    def start_recording(self, session_name=None):
        """Start data recording session"""
        with self.lock:
            if session_name is None:
                session_name = f"line_follow_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
            
            self.session_name = session_name
            self.is_recording = True
            self.start_time = time.time()
            
            # Clear previous data
            self.clear_data()
            
            logger.info(f"Started recording session: {session_name}")
    
    def stop_recording(self):
        """Stop data recording session"""
        with self.lock:
            self.is_recording = False
            logger.info(f"Stopped recording session: {self.session_name}")
    
    def add_data_point(self, line_position, pid_error, pid_output, left_speed, right_speed, 
                       active_sensors_list, setpoint=8.5):
        """Add a data point to the collection"""
        if not self.is_recording:
            return
            
        with self.lock:
            current_time = time.time()
            relative_time = current_time - self.start_time if self.start_time else 0
            
            self.timestamps.append(relative_time)
            self.line_positions.append(line_position)
            self.pid_errors.append(pid_error)
            self.pid_outputs.append(pid_output)
            self.left_motor_speeds.append(left_speed)
            self.right_motor_speeds.append(right_speed)
            self.active_sensors.append(active_sensors_list.copy() if active_sensors_list else [])
            self.setpoints.append(setpoint)
    
    def get_data_for_plotting(self):
        """Get current data for real-time plotting"""
        with self.lock:
            return {
                'timestamps': list(self.timestamps),
                'line_positions': list(self.line_positions),
                'pid_errors': list(self.pid_errors),
                'pid_outputs': list(self.pid_outputs),
                'left_motor_speeds': list(self.left_motor_speeds),
                'right_motor_speeds': list(self.right_motor_speeds),
                'active_sensors': list(self.active_sensors),
                'setpoints': list(self.setpoints)
            }
    
    def clear_data(self):
        """Clear all stored data"""
        with self.lock:
            self.timestamps.clear()
            self.line_positions.clear()
            self.pid_errors.clear()
            self.pid_outputs.clear()
            self.left_motor_speeds.clear()
            self.right_motor_speeds.clear()
            self.active_sensors.clear()
            self.setpoints.clear()
    
    def export_to_csv(self, filename=None):
        """Export collected data to CSV file"""
        if filename is None:
            filename = f"{self.session_name}.csv" if self.session_name else "line_follow_data.csv"
        
        with self.lock:
            try:
                with open(filename, 'w', newline='') as csvfile:
                    fieldnames = ['timestamp', 'line_position', 'setpoint', 'pid_error', 
                                'pid_output', 'left_motor_speed', 'right_motor_speed', 'active_sensors']
                    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                    
                    writer.writeheader()
                    for i in range(len(self.timestamps)):
                        writer.writerow({
                            'timestamp': self.timestamps[i],
                            'line_position': self.line_positions[i],
                            'setpoint': self.setpoints[i],
                            'pid_error': self.pid_errors[i],
                            'pid_output': self.pid_outputs[i],
                            'left_motor_speed': self.left_motor_speeds[i],
                            'right_motor_speed': self.right_motor_speeds[i],
                            'active_sensors': json.dumps(self.active_sensors[i])
                        })
                
                logger.info(f"Data exported to {filename}")
                return filename
            except Exception as e:
                logger.error(f"Failed to export data: {e}")
                return None

# Simple data collector for easy integration
class SimpleDataCollector:
    """Lightweight data collector that can be easily integrated"""
    
    def __init__(self):
        self.data = []
        self.is_recording = False
        self.start_time = None
        
    def start_recording(self):
        """Start collecting data"""
        self.is_recording = True
        self.start_time = time.time()
        self.data = []
        print("Data collection started")
        
    def stop_recording(self):
        """Stop collecting data"""
        self.is_recording = False
        print(f"Data collection stopped. Collected {len(self.data)} data points")
        
    def add_data(self, line_position, pid_error, pid_output, left_speed, right_speed, active_sensors):
        """Add a data point"""
        if not self.is_recording:
            return
            
        timestamp = time.time() - self.start_time
        self.data.append({
            'timestamp': timestamp,
            'line_position': line_position,
            'pid_error': pid_error,
            'pid_output': pid_output,
            'left_speed': left_speed,
            'right_speed': right_speed,
            'active_sensors': active_sensors,
            'setpoint': 8.5
        })
    
    def export_csv(self, filename=None):
        """Export data to CSV"""
        if not filename:
            filename = f"line_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            
        with open(filename, 'w', newline='') as f:
            if self.data:
                writer = csv.DictWriter(f, fieldnames=self.data[0].keys())
                writer.writeheader()
                writer.writerows(self.data)
        print(f"Data exported to {filename}")
        return filename