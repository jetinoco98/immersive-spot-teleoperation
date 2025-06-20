import os
import pandas as pd
import matplotlib.pyplot as plt

def analyze_turn_log(filename):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    logs_dir = os.path.join(script_dir, "logs")
    file_path = os.path.join(logs_dir, filename)

    df = pd.read_csv(file_path)
    df['timestamp'] -= df['timestamp'].min()  # Normalize time

    plt.figure(figsize=(12, 6))
    plt.title(f"Sensor Data: {os.path.basename(file_path)}")
    plt.plot(df['timestamp'], df['forward_velocity'], label='Forward Velocity')
    plt.plot(df['timestamp'], df['angular_velocity'], label='Angular Velocity')
    plt.xlabel("Time (s)")
    plt.ylabel("Values")
    plt.legend()
    plt.grid(True)
    plt.show()


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

    # --- 2. SPEED CONSISTENCY (using raw forward velocity) ---
    fwd_velocity = df['forward_velocity']
    speed_mean = fwd_velocity.mean()
    speed_std = fwd_velocity.std()

    zero_threshold = 0.01  # ~Zero movement
    zero_count = (fwd_velocity.abs() < zero_threshold).sum()
    total_count = len(fwd_velocity)
    zero_percentage = (zero_count / total_count) * 100

    print(f"\nSpeed Consistency:")
    print(f"  - Mean Forward Velocity: {speed_mean:.3f} m/s")
    print(f"  - Std Dev: {speed_std:.3f}")
    print(f"  - Zero-Speed Frames: {zero_count} / {total_count}")
    print(f"  - Zero-Speed Time: {zero_percentage:.2f}%")

    # --- Optional Plot ---
    plt.figure(figsize=(12, 6))
    plt.title("Forward Motion Test - Velocity & Yaw")
    plt.plot(df['timestamp'], df['forward_velocity'], label='Forward Velocity (m/s)', color='blue')
    plt.plot(df['timestamp'], df['yaw'], label='Yaw (°)', color='orange', alpha=0.6)
    plt.axhline(0, color='black', linestyle='--', alpha=0.4)
    plt.xlabel("Time (s)")
    plt.ylabel("Value")
    plt.ylim(-6, 6)
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    pass