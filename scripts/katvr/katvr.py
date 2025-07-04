import socket
import struct
import threading
import time
import math
import zmq
import json
from data_logger import DataLogger


# --- CONSTANTS 
VELOCITY_THRESHOLD = 1.0    # The minimum velocity to be considered as forward movement
FORWARD_VELOCITY_CHANGE_LIMIT = 3.5
FORWARD_VELOCITY_NORMAL = 0.4           
FORWARD_VELOCITY_HIGH = 0.5             
LATERAL_VELOCITY_BASE = 0.25

# --- Global logger instance 
name_logger = "katvr_log_" + time.strftime("%Y%m%d_%H%M%S")
logger = DataLogger(name_logger)

# ======================================================================
#                           KATVR DEVICE CLASS
# ======================================================================

class KATVR:
    """
    This class represents the KATVR device properties and methods for processing data.
    It handles yaw tracking, velocity smoothing, and state management for walking logic.
    """
    def __init__(self):
        # Yaw tracking
        self.previous_yaw_virtual = None
        self.yaw_virtual = 0
        self.previous_yaw = None
        self.yaw = 0
        # Raw velocity from device
        self.velocity = 0
        self.filtered_velocity = 0
        self.velocity_smoothing_factor = 0.2  # 0 < alpha < 1; lower = smoother
        # Velocity components
        self.forward_velocity = 0
        self.lateral_velocity = 0
        # Time and angular velocity
        self.delta_time = None
        self.angular_velocity = 0
        # Device state
        self.is_active = False
        self.last_update_time = None
        # Walking Logic State Machine (Normal, Walking Left, Walking Right)
        self.state = 'normal'
        self.last_state_change_time = time.time()
        self.yaw_temporal_offset = 0.0


    @staticmethod
    def normalize_angle(angle):
        """Normalizes an angle to the range [-180, 180) degrees."""
        return (angle + 180) % 360 - 180


    def smooth_velocity(self):
        """Applies low-pass filter to raw velocity to smooth step transitions."""
        alpha = self.velocity_smoothing_factor
        self.filtered_velocity = alpha * self.velocity + (1 - alpha) * self.filtered_velocity 
        if abs(self.filtered_velocity) < 0.01:
            self.filtered_velocity = 0.0


    def compute_velocity_components(self):
        """Computes forward/lateral velocities based on current state using predefined constants."""
        self.forward_velocity = 0
        self.lateral_velocity = 0

        if self.state == 'normal':
            if abs(self.filtered_velocity) < VELOCITY_THRESHOLD:
                self.forward_velocity = 0.0
            elif VELOCITY_THRESHOLD <= abs(self.filtered_velocity) < FORWARD_VELOCITY_CHANGE_LIMIT:
                self.forward_velocity = FORWARD_VELOCITY_NORMAL if self.filtered_velocity > 0 else -FORWARD_VELOCITY_NORMAL
            else:
                self.forward_velocity = FORWARD_VELOCITY_HIGH if self.filtered_velocity > 0 else -FORWARD_VELOCITY_HIGH

        elif self.state == 'walking_right':
            if abs(self.filtered_velocity) >= VELOCITY_THRESHOLD:
                self.lateral_velocity = -LATERAL_VELOCITY_BASE

        elif self.state == 'walking_left':
            if abs(self.filtered_velocity) >= VELOCITY_THRESHOLD:
                self.lateral_velocity = LATERAL_VELOCITY_BASE


    def compute_angular_velocity(self):
        """Computes angular velocity using yaw changes."""
        if self.previous_yaw is None:
            self.angular_velocity = 0.0
            return

        delta_angle = self.normalize_angle(self.yaw - self.previous_yaw)
        angular_velocity_deg = delta_angle / self.delta_time
        self.angular_velocity = math.radians(angular_velocity_deg)


    def update_actual_yaw_and_state(self):
        """
        Detects virtual yaw jumps and updates actual yaw and walking state.

        This is needed because when the user activates lateral walking, the KATVR device simply
        shifts the virtual yaw to the left or right by 90 degrees.
        """
        YAW_JUMP_THRESHOLD = 50  # degrees
        WALKING_STATE_MIN_DURATION = 0.5  # seconds

        if self.previous_yaw_virtual is None:
            self.yaw = self.yaw_virtual
            return
        
        # Calculate the change in yaw
        delta_angle = self.yaw_virtual - self.previous_yaw_virtual
        delta_angle = self.normalize_angle(delta_angle)
        current_time = time.time()

        # Check for jumps
        if abs(delta_angle) >= YAW_JUMP_THRESHOLD:
            if self.state == "normal":
                if delta_angle > 0:
                    new_state = "walking_left"
                    self.yaw_temporal_offset = -90
                else:
                    new_state = "walking_right"
                    self.yaw_temporal_offset = 90
            elif self.state in ["walking_left", "walking_right"]:
                new_state = "normal"

            if self.state != new_state:
                self.state = new_state
                self.last_state_change_time = current_time

        # Exit walking state after minimum duration if velocity low
        if self.state in ["walking_left", "walking_right"]:
            if current_time - self.last_state_change_time >= WALKING_STATE_MIN_DURATION:
                if abs(self.filtered_velocity) < VELOCITY_THRESHOLD:
                    self.state = "normal"
                    self.yaw_temporal_offset = 0.0
                    self.last_state_change_time = current_time

        # Update yaw based on state
        if self.state == "normal":
            self.yaw = self.yaw_virtual
            self.yaw_temporal_offset = 0.0
        else:
            self.yaw = self.normalize_angle(self.yaw_virtual + self.yaw_temporal_offset)


    def process_internal_values(self):
        """Main update loop for processing KATVR values."""
        self.last_update_time = time.time()
        self.smooth_velocity()
        self.update_actual_yaw_and_state()
        self.compute_velocity_components()
        self.compute_angular_velocity()
        self.previous_yaw_virtual = self.yaw_virtual
        self.previous_yaw = self.yaw


# ======================================================================
#                           KATVR MANAGER CLASS
# ======================================================================

class KATVRManager:
    """Manages the KATVR device connection, and handles the data flow between entry (UDP), 
    processing (KATVR class), and exit (ZMQ) points."""
    def __init__(self, zmq_address, udp_ip, udp_port):
        self.katvr = KATVR()
        # ZMQ setup
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.connect(zmq_address)
        # UDP setup
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        self.buffer_size = 1024
        
    # ================ ENTRY: INCOMING UDP MESSAGES ================

    def message_handler(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((self.udp_ip, self.udp_port))
        print(f"🔵 Listening for UDP float arrays on {self.udp_ip}:{self.udp_port}...")

        while True:
            data, addr = sock.recvfrom(self.buffer_size)
            if len(data) % 4 != 0:
                print(f"⚠️ Incompatible data from {addr}.")
                break
            float_array = struct.unpack(f'{len(data) // 4}f', data)
            if len(float_array) < 3:
                print(f"⚠️ Expected at least 3 floats, got: {float_array}")
                break
            self._save_katvr_data(float_array)

        print("🔴 UDP listener has stopped.")

    def _save_katvr_data(self, float_array):
        if not self.katvr.is_active:
            self.katvr.is_active = True
            print("✅ KATVR connection established.")

        self.katvr.delta_time = float_array[0]
        self.katvr.yaw_virtual = float_array[1]
        self.katvr.velocity = float_array[2]
        self.katvr.process_internal_values()
        logger.log(self.katvr)
        self.send_to_oculus_client()

    # ================ SENDING DATA THROUGH ZMQ ================

    def send_to_oculus_client(self):
        data = {
            "yaw": round(self.katvr.yaw, 2),
            "forward_velocity": round(self.katvr.forward_velocity, 2),
            "lateral_velocity": round(self.katvr.lateral_velocity, 2),
        }
        payload = json.dumps(data).encode("utf-8")
        self.socket.send_string("from_katvr", zmq.SNDMORE)
        self.socket.send(payload)

    # ================ MONITORING LOOP ================

    def monitor_loop(self, update_interval=1/20):
        """
        Continuously monitors the KATVR state and prints updates to the console.
        """
        while True:
            if not self.katvr.is_active:
                time.sleep(0.1)
                continue

            if self.katvr.last_update_time is not None and time.time() - self.katvr.last_update_time > 0.2:
                print("⚠️  Disconnected. Waiting for new data..." + " " * 50, end="\r")
            else:
                print(
                    f"VirtYaw:{self.katvr.yaw_virtual:.1f}° (Yaw:{self.katvr.yaw:.1f}°) | State:{self.katvr.state} | "
                    f"Vel:{self.katvr.velocity:.2f} (Fwd:{self.katvr.forward_velocity:.2f} Lat:{self.katvr.lateral_velocity:.2f})"
                    + " " * 20,  # padding spaces to clear previous chars
                    end="\r",
                    flush=True
                )
            time.sleep(update_interval)


# ======================================================================
#                                MAIN
# ======================================================================

if __name__ == "__main__":
    katvr_manager = KATVRManager(zmq_address="tcp://localhost:5555", udp_ip="127.0.0.1", udp_port=8002)
    threading.Thread(target=katvr_manager.message_handler, daemon=True).start()

    try:
        katvr_manager.monitor_loop()
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        logger.close()
