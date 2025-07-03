import socket
import struct
import threading
import time
import math
import zmq
import json
from data_logger import DataLogger


# --- CONSTANTS ---
VELOCITY_THRESHOLD = 1.0                # The minimum velocity to be considered as forward movement
FORWARD_VELOCITY_CHANGE_LIMIT = 3.0     # The velocity limit above which the forward velocity is considered high
FORWARD_VELOCITY_NORMAL = 0.4           # NORMAL SPEED
FORWARD_VELOCITY_HIGH = 0.5             # HIGH SPEED
# ---
LATERAL_VELOCITY_VALUE = 0.25           # A transformed value to be used for lateral velocity

# --- TESTING MODE ---
# True: Testing mode. Performs an interactive test sequence.
# False: Normal operation. Sends the data to the Oculus client.
TESTING_MODE = False  

# ======================================================================
#                           KATVR DEVICE CLASS
# ======================================================================

class KATVR:
    """Represents the KATVR device properties and methods for processing data, including motion state tracking."""
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


    def smooth_velocity(self):
        """Applies low-pass filter to raw velocity to smooth step transitions."""
        alpha = self.velocity_smoothing_factor
        self.filtered_velocity = alpha * self.velocity + (1 - alpha) * self.filtered_velocity 


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
                self.lateral_velocity = -LATERAL_VELOCITY_VALUE

        elif self.state == 'walking_left':
            if abs(self.filtered_velocity) >= VELOCITY_THRESHOLD:
                self.lateral_velocity = LATERAL_VELOCITY_VALUE


    def compute_angular_velocity(self):
        """Computes angular velocity using virtual yaw changes."""
        if self.previous_yaw is None:
            self.angular_velocity = 0.0
            return

        delta_angle = (self.yaw - self.previous_yaw + 180) % 360 - 180
        angular_velocity_deg = delta_angle / self.delta_time
        self.angular_velocity = math.radians(angular_velocity_deg)


    def update_actual_yaw_and_state(self):
        """Detects virtual yaw jumps and updates actual yaw and walking state."""
        if self.previous_yaw_virtual is None:
            return
        
        # When the filtered velocity is less than half the threshold, state must be normal
        if abs(self.filtered_velocity) < (VELOCITY_THRESHOLD / 2):
            self.state = 'normal'
            self.forward_velocity = 0.0
            self.lateral_velocity = 0.0

        # Calculate the change in yaw
        delta_angle = self.yaw_virtual - self.previous_yaw_virtual
        delta_angle = (delta_angle + 180) % 360 - 180

        # Check for jumps
        if abs(delta_angle) >= 60:
            if self.state == "normal":
                if delta_angle > 0:
                    self.state = "walking_left"
                else:
                    self.state = "walking_right" 
            elif self.state == "walking_left" or self.state == "walking_right":
                self.state = "normal"
        
        if self.state == "normal":
            self.yaw = self.yaw_virtual


    def process_internal_values(self):
        """Main update loop for processing KATVR values."""
        self.last_update_time = time.time() 
        self.compute_angular_velocity()
        self.update_actual_yaw_and_state()
        self.smooth_velocity()
        self.compute_velocity_components()
        self.previous_yaw_virtual = self.yaw_virtual
        self.previous_yaw = self.yaw


# ======================================================================
#                           KATVR MANAGER CLASS
# ======================================================================

class KATVRManager:
    """Manages the KATVR device connection and data processing."""
    def __init__(self, zmq_address, udp_ip, udp_port):
        self.katvr = KATVR()
        self.testing_mode = TESTING_MODE
        # ZMQ setup
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.connect(zmq_address)
        # UDP setup
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        self.buffer_size = 1024
        
    # ================ HANDLING INCOMING UDP MESSAGES ================

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
            self._process_katvr_data(float_array)

        print("🔴 UDP listener has stopped.")

    def _process_katvr_data(self, float_array):
        if not self.katvr.is_active:
            self.katvr.is_active = True
            print("✅ KATVR connection established.")

        self.katvr.delta_time = float_array[0]
        self.katvr.yaw_virtual = float_array[1]
        self.katvr.velocity = float_array[2]
        self.katvr.process_internal_values()

        if not self.testing_mode:
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

    def monitor_loop(self, update_interval=0.05):
        """Continuously prints KATVR sensor data."""
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

    # ================ TESTING FUNCTIONS ================

    def run_test(self, test_name, duration_seconds, countdown_seconds=5):
        logger = DataLogger(test_name)
        print(f"\n--- {test_name.upper()} ---")

        def countdown(seconds, label=""):
            for i in range(seconds, 0, -1):
                print(f"{label} {i}...", end='\r')
                time.sleep(1)
            print(' ' * 30, end='\r')  # Clear line

        # Get Ready Countdown
        countdown(countdown_seconds, label="Get ready:")

        # Test Active Countdown
        print("Start now!")
        start_time = time.time()
        while time.time() - start_time < duration_seconds:
            time_remaining = int(duration_seconds - (time.time() - start_time))
            print(f"Recording: {time_remaining} seconds remaining...", end='\r')
            if self.katvr.is_active:
                logger.log(self.katvr)
            time.sleep(0.05)

        logger.close()
        print(' ' * 50, end='\r')  # Clear final countdown
        print(f"Test '{test_name}' complete. Data saved to: {logger.file_path}\n")
        return logger.file_path
    
    def start_interactive_tests(self):
        """Runs the interactive test sequence using KATVR data."""
        print("\n=== KATVR SENSOR TEST SEQUENCE ===\n")

        input("TEST 1. Prepare to turn in place. Press ENTER when ready...")
        self.run_test("turning_in_place", 60, 10)

        input("TEST 2. Prepare to walk. Press ENTER when ready...")
        self.run_test("walking", 60, 10)

        input("TEST 3. Prepare to walk laterally. Press ENTER when ready...")
        self.run_test("lateral_movement", 30, 5)

        print("\n=== TEST SEQUENCE ENDED ===")


# ======================================================================
#                                MAIN
# ======================================================================

if __name__ == "__main__":
    katvr_manager = KATVRManager(zmq_address="tcp://localhost:5555", udp_ip="127.0.0.1", udp_port=8002)
    threading.Thread(target=katvr_manager.message_handler, daemon=True).start()

    try:
        if katvr_manager.testing_mode:
            katvr_manager.start_interactive_tests()
        else:
            katvr_manager.monitor_loop()
    except KeyboardInterrupt:
        print("\nExiting...")
