import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

import os
import pandas as pd
import matplotlib.pyplot as plt


def analyze_turn_log(filename):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    logs_dir = os.path.join(script_dir, "logs")
    file_path = os.path.join(logs_dir, filename)

    df = pd.read_csv(file_path)
    df['timestamp'] -= df['timestamp'].min()  # Normalize time

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6), sharex=True)
    fig.suptitle(f"Sensor Data: {os.path.basename(file_path)}")

    # Left subplot: Raw and filtered velocity
    ax1.plot(df['timestamp'], df['velocity'], label='Raw Velocity', color='red', alpha=0.7, linewidth=1)
    ax1.plot(df['timestamp'], df['filtered_velocity'], label='Filtered Velocity', color='blue', linewidth=2)
    ax1.set_title("Raw vs Filtered Velocity")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Velocity")
    ax1.legend()
    ax1.grid(True)

    # Right subplot: Forward and lateral velocity
    ax2.plot(df['timestamp'], df['forward_velocity'], label='Forward Velocity', color='green')
    ax2.plot(df['timestamp'], df['lateral_velocity'], label='Lateral Velocity', color='purple')
    ax2.set_title("Directional Velocity")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Velocity")
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout(rect=[0, 0, 1, 0.95])
    plt.show()

# =======================================================================

def analyze_forward_motion_log(filename):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    logs_dir = os.path.join(script_dir, "logs")
    file_path = os.path.join(logs_dir, filename)

    df = pd.read_csv(file_path)
    df['timestamp'] -= df['timestamp'].min()  # Normalize time

    print(f"\n=== ANALYSIS: {os.path.basename(file_path)} ===")

    # --- 1. SWAY ANALYSIS ---
    yaw_range = df['yaw'].max() - df['yaw'].min()
    yaw_std = df['yaw'].std()
    print(f"\nSway Analysis (Yaw):")
    print(f"  - Yaw Range: {yaw_range:.2f}°")
    print(f"  - Yaw Std Dev: {yaw_std:.2f}°")

    # --- 2. SPEED CONSISTENCY (using raw and filtered velocity) ---
    raw_velocity = df['velocity']
    filtered_velocity = df['filtered_velocity']
    
    raw_mean = raw_velocity.mean()
    raw_std = raw_velocity.std()
    filtered_mean = filtered_velocity.mean()
    filtered_std = filtered_velocity.std()

    zero_threshold = 0.01  # ~Zero movement
    zero_count_raw = (raw_velocity.abs() < zero_threshold).sum()
    zero_count_filtered = (filtered_velocity.abs() < zero_threshold).sum()
    total_count = len(raw_velocity)
    zero_percentage_raw = (zero_count_raw / total_count) * 100
    zero_percentage_filtered = (zero_count_filtered / total_count) * 100

    print(f"\nSpeed Consistency:")
    print(f"  - Raw Velocity - Mean: {raw_mean:.3f} m/s, Std Dev: {raw_std:.3f}")
    print(f"  - Filtered Velocity - Mean: {filtered_mean:.3f} m/s, Std Dev: {filtered_std:.3f}")
    print(f"  - Zero-Speed Frames (Raw): {zero_count_raw} / {total_count} ({zero_percentage_raw:.2f}%)")
    print(f"  - Zero-Speed Frames (Filtered): {zero_count_filtered} / {total_count} ({zero_percentage_filtered:.2f}%)")

    # --- Plot with Raw and Filtered Velocity Side by Side ---
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))
    
    # Left plot: Raw vs Filtered Velocity
    ax1.plot(df['timestamp'], df['velocity'], label='Raw Velocity', color='red', alpha=0.7, linewidth=1)
    ax1.plot(df['timestamp'], df['filtered_velocity'], label='Filtered Velocity', color='blue', linewidth=2)
    ax1.axhline(0, color='black', linestyle='--', alpha=0.4)
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Velocity (m/s)")
    ax1.set_title("Raw vs Filtered Velocity")
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Right plot: Yaw over time
    ax2.plot(df['timestamp'], df['yaw'], label='Yaw (°)', color='orange', linewidth=2)
    ax2.axhline(0, color='black', linestyle='--', alpha=0.4)
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Yaw (°)")
    ax2.set_title("Yaw Orientation")
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    plt.suptitle(f"Forward Motion Analysis: {os.path.basename(file_path)}")
    plt.tight_layout()
    plt.show()


def analyze_lateral_motion_log(filename):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    logs_dir = os.path.join(script_dir, "logs")
    file_path = os.path.join(logs_dir, filename)

    df = pd.read_csv(file_path)
    df['timestamp'] -= df['timestamp'].min()  # Normalize time

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6), sharex=True)
    fig.suptitle(f"Lateral Motion Analysis: {os.path.basename(file_path)}")

    # Left subplot: Lateral velocity
    ax1.plot(df['timestamp'], df['lateral_velocity'], label='Lateral Velocity', color='purple')
    ax1.set_title("Lateral Velocity")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Lateral Velocity (m/s)")
    ax1.legend()
    ax1.grid(True)

    # Right subplot: Virtual yaw
    if 'virtual_yaw' in df.columns:
        ax2.plot(df['timestamp'], df['virtual_yaw'], label='Virtual Yaw (°)', color='teal')
        ax2.set_ylabel("Virtual Yaw (°)")
    else:
        ax2.plot(df['timestamp'], df['yaw'], label='Yaw (°)', color='orange')
        ax2.set_ylabel("Yaw (°)")
    ax2.set_title("Virtual Yaw")
    ax2.set_xlabel("Time (s)")
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout(rect=[0, 0, 1, 0.95])
    plt.show()


if __name__ == "__main__":
    try:
        analyze_turn_log("katvr_log_turning_in_place.csv")
    except FileNotFoundError:
        print("Log file for turning in place not found. Please ensure the file exists in the logs directory.")

    # try:
    #     analyze_forward_motion_log("katvr_log_walking.csv")
    # except FileNotFoundError:
    #     print("Log file for walking forward not found. Please ensure the file exists in the logs directory.")

    # try:
    #     analyze_lateral_motion_log("katvr_log_lateral_movement.csv")
    # except FileNotFoundError:
    #     print("Log file for lateral movement not found. Please ensure the file exists in the logs directory.")
