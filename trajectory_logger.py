import csv
import time
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime
from collections import deque
import matplotlib.patches as patches

class OscillationAnalyzer:
    def __init__(self, window_size=100):
        """
        Advanced AGV Magnetic Sensor Oscillation Analyzer
        
        Args:
            window_size: Number of data points to keep in rolling window
        """
        self.start_time = time.time()
        self.data = []
        self.window_size = window_size
        self.rolling_data = deque(maxlen=window_size)
        
        # Sensor configuration (from your config)
        self.SENSOR_COUNT = 16
        self.CENTER_POSITION = 8.5  # Between sensors 8 and 9
        self.SENSOR_SPACING_MM = 10  # 10mm spacing
        self.SENSOR_SPACING_CM = 1.0  # 1cm per sensor unit
        
        # Statistics tracking
        self.stats = {
            'oscillations': 0,
            'left_bias_time': 0,
            'right_bias_time': 0,
            'max_left_dev': 0,
            'max_right_dev': 0,
            'avg_deviation': 0,
            'total_samples': 0,
            'last_crossing': 0
        }
        
        # Setup the comprehensive plot
        self.setup_plot()
        
        # CSV filename with European format
        self.filename = f"oscillation_analysis_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        
        print(f"🎯 Oscillation Analyzer initialized")
        print(f"📊 Sensor Array: 1-8 (RIGHT) | 9-16 (LEFT)")
        print(f"📍 Center Position: {self.CENTER_POSITION}")
        print(f"📁 Data will be saved to: {self.filename}")

    def setup_plot(self):
        """Setup the comprehensive oscillation analysis plot"""
        plt.ion()
        self.fig, self.ax1 = plt.subplots(figsize=(14, 8))
        
        # Primary Y-axis: Sensor Positions (1-16)
        self.ax1.set_ylim(0, 17)
        self.ax1.set_ylabel('Sensor Position (1-16)', fontsize=12, fontweight='bold')
        self.ax1.set_xlabel('Time (seconds)', fontsize=12, fontweight='bold')
        
        # Secondary Y-axis: Deviation in cm
        self.ax2 = self.ax1.twinx()
        self.ax2.set_ylim(-7.5, 7.5)  # ±7.5cm from center
        self.ax2.set_ylabel('Deviation from Center (cm)', fontsize=12, fontweight='bold', color='red')
        
        # Center line at 8.5
        self.ax1.axhline(y=self.CENTER_POSITION, color='black', linestyle='-', 
                        linewidth=2, label='CENTER LINE (8.5)', alpha=0.8)
        
        # Sensor zones
        self.ax1.axhspan(1, 8, color='lightblue', alpha=0.2, label='RIGHT SIDE (1-8)')
        self.ax1.axhspan(9, 16, color='lightcoral', alpha=0.2, label='LEFT SIDE (9-16)')
        
        # Sensor position markers
        for i in range(1, 17):
            self.ax1.axhline(y=i, color='gray', linestyle=':', alpha=0.3, linewidth=0.5)
            
        # Add sensor labels
        for i in [1, 4, 8, 9, 12, 16]:
            self.ax1.text(-0.5, i, f'S{i}', fontsize=8, ha='right', va='center')
        
        # Main oscillation line
        self.line_position, = self.ax1.plot([], [], 'b-', linewidth=2, 
                                           label='Line Position', alpha=0.8)
        
        # Deviation fill areas
        self.left_fill = None
        self.right_fill = None
        
        # Setup legends and titles
        self.ax1.set_title('AGV Magnetic Sensor Oscillation Analysis\n'
                          'Sensor Array: [1-8=RIGHT] | [9-16=LEFT] | Center=8.5', 
                          fontsize=14, fontweight='bold')
        
        # Position legends
        self.ax1.legend(loc='upper left', fontsize=10)
        
        # Add grid
        self.ax1.grid(True, alpha=0.3)
        
        # Statistics text box
        self.stats_text = self.ax1.text(0.02, 0.98, '', transform=self.ax1.transAxes,
                                       verticalalignment='top', fontsize=9,
                                       bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        # Sensor layout diagram (top right)
        self.ax1.text(0.98, 0.98, 
                     'SENSOR LAYOUT:\n[1][2][3][4][5][6][7][8] | [9][10][11][12][13][14][15][16]\n'
                     '       RIGHT SIDE         |         LEFT SIDE\n'
                     '                    CENTER (8.5)',
                     transform=self.ax1.transAxes, ha='right', va='top',
                     fontsize=8, fontfamily='monospace',
                     bbox=dict(boxstyle='round', facecolor='lightgray', alpha=0.7))

    def calculate_line_position(self, position_value):
        """
        Calculate line position from 16-bit sensor data
        
        Args:
            position_value: 16-bit value where each bit represents a sensor (inverted logic)
        
        Returns:
            line_position: Float value 1-16, or None if no line detected
        """
        if position_value is None:
            return None
            
        # Convert to active sensors (invert bits since 0 = active)
        active_sensors = []
        for i in range(16):
            if not (position_value & (1 << i)):  # Bit is 0 = sensor active
                active_sensors.append(i + 1)  # Sensors numbered 1-16
        
        if not active_sensors:
            return None
            
        if len(active_sensors) > 8:  # Too many sensors (noise)
            return None
            
        # Calculate weighted average position
        line_position = sum(active_sensors) / len(active_sensors)
        
        return line_position

    def update(self, line_position, raw_sensor_value=None, pid_error=None, 
               pid_correction=None, left_speed=None, right_speed=None, 
               pid_components=None, line_detected=True, dt=0.05):
        """
        Update oscillation analysis with new sensor data
        
        Args:
            line_position: Calculated line position (1-16)
            raw_sensor_value: Raw 16-bit sensor data
            Other parameters: For compatibility with existing code
        """
        current_time = time.time() - self.start_time
        
        # If line_position is None, try to calculate from raw sensor data
        if line_position is None and raw_sensor_value is not None:
            line_position = self.calculate_line_position(raw_sensor_value)
        
        if line_position is None:
            return  # Skip this update
            
        # Calculate deviation from center in cm
        deviation_cm = (line_position - self.CENTER_POSITION) * self.SENSOR_SPACING_CM
        
        # Update statistics
        self.update_statistics(current_time, line_position, deviation_cm)
        
        # Store data
        data_point = {
            'time': current_time,
            'line_position': line_position,
            'deviation_cm': deviation_cm,
            'raw_sensor': raw_sensor_value,
            'line_detected': line_detected,
            'pid_error': pid_error,
            'pid_correction': pid_correction
        }
        
        self.data.append(data_point)
        self.rolling_data.append(data_point)
        
        # Update plot every few samples for performance
        if len(self.data) % 3 == 0:
            self.update_plot()

    def update_statistics(self, current_time, line_position, deviation_cm):
        """Update oscillation statistics"""
        self.stats['total_samples'] += 1
        
        # Track maximum deviations
        if deviation_cm < 0:  # LEFT deviation
            self.stats['max_left_dev'] = min(self.stats['max_left_dev'], deviation_cm)
            self.stats['left_bias_time'] += 0.05  # Approximate dt
        else:  # RIGHT deviation
            self.stats['max_right_dev'] = max(self.stats['max_right_dev'], deviation_cm)
            self.stats['right_bias_time'] += 0.05
        
        # Count oscillations (zero crossings)
        if len(self.data) > 1:
            prev_deviation = self.data[-1]['deviation_cm']
            if (prev_deviation < 0 and deviation_cm >= 0) or (prev_deviation >= 0 and deviation_cm < 0):
                self.stats['oscillations'] += 1
                self.stats['last_crossing'] = current_time
        
        # Calculate running average deviation
        if self.rolling_data:
            total_abs_dev = sum(abs(d['deviation_cm']) for d in self.rolling_data)
            self.stats['avg_deviation'] = total_abs_dev / len(self.rolling_data)

    def update_plot(self):
        """Update the oscillation plot with latest data"""
        if len(self.rolling_data) < 2:
            return
        
        # Extract data for plotting
        times = [d['time'] for d in self.rolling_data]
        positions = [d['line_position'] for d in self.rolling_data]
        deviations = [d['deviation_cm'] for d in self.rolling_data]
        
        # Update main line
        self.line_position.set_data(times, positions)
        
        # Clear previous fill areas
        for collection in self.ax1.collections:
            if hasattr(collection, '_is_fill'):
                collection.remove()
        
        # Add colored fill areas for deviations
        times_array = np.array(times)
        deviations_array = np.array(deviations)
        
        # Fill areas above and below center
        self.ax1.fill_between(times_array, self.CENTER_POSITION, 
                             self.CENTER_POSITION + deviations_array/self.SENSOR_SPACING_CM,
                             where=(deviations_array >= 0), color='red', alpha=0.3, 
                             interpolate=True, label='RIGHT Deviation')
        self.ax1.fill_between(times_array, self.CENTER_POSITION,
                             self.CENTER_POSITION + deviations_array/self.SENSOR_SPACING_CM,
                             where=(deviations_array < 0), color='blue', alpha=0.3,
                             interpolate=True, label='LEFT Deviation')
        
        # Mark fill areas for removal
        for collection in self.ax1.collections[-2:]:
            collection._is_fill = True
        
        # Update axis limits
        if times:
            time_range = max(times) - min(times)
            self.ax1.set_xlim(max(0, max(times) - max(30, time_range)), max(times) + 1)
        
        # Update statistics display
        self.update_stats_display()
        
        # Redraw
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def update_stats_display(self):
        """Update the statistics text box"""
        total_time = self.stats['left_bias_time'] + self.stats['right_bias_time']
        
        if total_time > 0:
            left_percentage = (self.stats['left_bias_time'] / total_time) * 100
            right_percentage = (self.stats['right_bias_time'] / total_time) * 100
        else:
            left_percentage = right_percentage = 0
        
        stats_text = f"""📊 OSCILLATION STATISTICS
🎯 Samples: {self.stats['total_samples']}
🔄 Oscillations: {self.stats['oscillations']}
📏 Avg Deviation: {self.stats['avg_deviation']:.2f} cm
⬅️  Max LEFT: {self.stats['max_left_dev']:.2f} cm
➡️  Max RIGHT: {self.stats['max_right_dev']:.2f} cm
🔵 LEFT Bias: {left_percentage:.1f}%
🔴 RIGHT Bias: {right_percentage:.1f}%
⏱️  Last Crossing: {self.stats['last_crossing']:.1f}s ago"""
        
        self.stats_text.set_text(stats_text)

    def save_to_csv(self):
        """Save oscillation data to CSV with European format"""
        if not self.data:
            print("❌ No data to save!")
            return
        
        # European CSV format (semicolon separator, comma for decimals)
        with open(self.filename, 'w', newline='', encoding='utf-8') as f:
            # Write header
            f.write('time;line_position;deviation_cm;raw_sensor;line_detected;pid_error;pid_correction\n')
            
            # Write data with European decimal format
            for d in self.data:
                line = f"{d['time']:.3f}".replace('.', ',') + ';'
                line += f"{d['line_position']:.3f}".replace('.', ',') + ';'
                line += f"{d['deviation_cm']:.3f}".replace('.', ',') + ';'
                line += f"{d['raw_sensor'] or 0};{d['line_detected']};{d['pid_error'] or 0:.3f}".replace('.', ',') + ';'
                line += f"{d['pid_correction'] or 0:.3f}".replace('.', ',') + '\n'
                f.write(line)
        
        # Save statistics summary
        stats_filename = self.filename.replace('.csv', '_summary.txt')
        with open(stats_filename, 'w', encoding='utf-8') as f:
            f.write("🎯 AGV OSCILLATION ANALYSIS SUMMARY\n")
            f.write("=" * 50 + "\n\n")
            f.write(f"📊 Total Samples: {self.stats['total_samples']}\n")
            f.write(f"🔄 Total Oscillations: {self.stats['oscillations']}\n")
            f.write(f"📏 Average Deviation: {self.stats['avg_deviation']:.3f} cm\n")
            f.write(f"⬅️  Maximum LEFT Deviation: {self.stats['max_left_dev']:.3f} cm\n")
            f.write(f"➡️  Maximum RIGHT Deviation: {self.stats['max_right_dev']:.3f} cm\n")
            f.write(f"🔵 LEFT Bias Time: {self.stats['left_bias_time']:.2f} seconds\n")
            f.write(f"🔴 RIGHT Bias Time: {self.stats['right_bias_time']:.2f} seconds\n")
            
            if self.data:
                total_time = self.data[-1]['time']
                f.write(f"⏱️  Total Analysis Time: {total_time:.2f} seconds\n")
                
                if total_time > 0:
                    osc_rate = self.stats['oscillations'] / total_time * 60
                    f.write(f"📈 Oscillation Rate: {osc_rate:.1f} oscillations/minute\n")
        
        print(f"✅ Oscillation data saved to: {self.filename}")
        print(f"📋 Summary saved to: {stats_filename}")

    def finalize(self):
        """Finalize analysis and save data"""
        self.save_to_csv()
        
        print(f"\n🎯 OSCILLATION ANALYSIS COMPLETE")
        print(f"📊 Total Oscillations: {self.stats['oscillations']}")
        print(f"📏 Average Deviation: {self.stats['avg_deviation']:.2f} cm")
        print(f"⬅️  Max LEFT: {self.stats['max_left_dev']:.2f} cm")
        print(f"➡️  Max RIGHT: {self.stats['max_right_dev']:.2f} cm")
        
        plt.ioff()
        plt.show()

# Factory function for easy replacement
class TrajectoryLogger(OscillationAnalyzer):
    """Alias for backward compatibility"""
    pass