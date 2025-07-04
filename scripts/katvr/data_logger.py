import os
import csv
import time
from datetime import datetime

class DataLogger:
    def __init__(self, test_name):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        base_filename = f"katvr_log_{test_name}"
        ext = ".csv"

        script_dir = os.path.dirname(os.path.abspath(__file__))
        logs_dir = os.path.join(script_dir, "logs")
        os.makedirs(logs_dir, exist_ok=True)

        # Resolve filename conflict
        n = 0
        while True:
            suffix = f"_{n}" if n > 0 else ""
            filename = f"{base_filename}{suffix}{ext}"
            self.file_path = os.path.join(logs_dir, filename)
            if not os.path.exists(self.file_path):
                break
            n += 1

        self.fields = ["timestamp", "yaw", "yaw_virtual", "velocity", "filtered_velocity",
                       "forward_velocity", "lateral_velocity", "angular_velocity", "state"]
        self.file = open(self.file_path, mode="w", newline='')
        self.writer = csv.DictWriter(self.file, fieldnames=self.fields)
        self.writer.writeheader()

    def log(self, katvr):
        self.writer.writerow({
            "timestamp": time.time(),
            "yaw": round(katvr.yaw, 3),
            "yaw_virtual": round(katvr.yaw_virtual, 3),
            "velocity": round(katvr.velocity, 3),
            "filtered_velocity": round(katvr.filtered_velocity, 3),
            "forward_velocity": round(katvr.forward_velocity, 3),
            "lateral_velocity": round(katvr.lateral_velocity, 3),
            "angular_velocity": round(katvr.angular_velocity, 3),
            "state": katvr.state,
        })

    def close(self):
        self.file.close()
