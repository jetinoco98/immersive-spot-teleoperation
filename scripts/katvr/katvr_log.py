# Retrieve the output values of the controllers in katvr_main.py and log to csv

import time
import csv
from datetime import datetime
import os

class DataLogger:
    def __init__(self, log_dir="logs"):
        self.log_dir = log_dir
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        
        # Create a new log file with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_file = os.path.join(log_dir, f"controller_log_{timestamp}.csv")
        
        # Initialize CSV file with headers
        with open(self.log_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp',
                'controller_type',
                'control_signal',
                'raw_angular_velocity',
                'smoothed_angular_velocity_rad',
                'error',
                'yaw',
                'move_speed'
            ])
    
    def log_open_loop_data(self, control_signal, raw_angular_velocity, smoothed_angular_velocity_rad, yaw, move_speed):
        with open(self.log_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f"),
                'open_loop',
                control_signal,
                raw_angular_velocity,
                smoothed_angular_velocity_rad,
                '',  # No error in open loop
                yaw,
                move_speed
            ])
    
    def log_closed_loop_data(self, control_signal, error, yaw, move_speed):
        with open(self.log_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f"),
                'closed_loop',
                control_signal,
                '',  # No raw angular velocity in closed loop
                '',  # No smoothed angular velocity in closed loop
                error,
                yaw,
                move_speed
            ])
