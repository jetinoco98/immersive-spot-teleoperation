from pythonosc import dispatcher, osc_server
import threading
import time
import math
import zmq
import json
import os
from data_logger import DataLogger


# --- CONSTANTS ---
FORWARD_VELOCITY_THRESHOLD = 1.5        # The minimum velocity obtained from KATVR to be considered as forward movement
FORWARD_VELOCITY_CHANGE_LIMIT = 2.5     # The velocity limit above which the forward velocity is considered high
FORWARD_VELOCITY_NORMAL = 0.25          # NORMAL SPEED
FORWARD_VELOCITY_HIGH = 0.5             # HIGH SPEED

TESTING_MODE = False  
# False: Normal operation. Sends the data to the Oculus client.
# True: Testing mode. Performs an interactive test sequence.


# --- CLASSES ---
class KATVR:
    """Represents the KATVR device properties and methods for processing data."""
    def __init__(self):
        # KATVR device properties
        self.previous_yaw = None
        self.uncorrected_yaw = 0
        self.yaw = 0
        self.current_delta_time = None
        self.forward_velocity = 0
        self.forward_velocity_normalized = 0
        self.angular_velocity = 0
        self.angular_velocity_clamped = 0  
        # Additional properties
        self.is_active = False
        self.last_message_time = None
        self.message_frequencies = []
        logs_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
        os.makedirs(logs_dir, exist_ok=True)
        self.frequency_log_path = os.path.join(logs_dir, "katvr_frequency_log.txt")

    # --- INTERNAL METHODS ---
    def apply_special_yaw_correction(self):
        """
        Corrects the yaw value obtained from KATVR by applying an offset of 90°.
        Updates the self.yaw property.
        """
        offset = 90
        self.yaw = (self.uncorrected_yaw - offset + 180) % 360 - 180

    def compute_forward_velocity_normalized(
        self,
        threshold=FORWARD_VELOCITY_THRESHOLD,
        velocity_change_limit=FORWARD_VELOCITY_CHANGE_LIMIT,
        normal_velocity=FORWARD_VELOCITY_NORMAL,
        high_velocity=FORWARD_VELOCITY_HIGH
    ):
        """ 
        Normalizes the forward velocity from the KATVR device to a range suitable for the Spot robot.
        Updates the self.forward_velocity_normalized property.
        """
        if abs(self.forward_velocity) < threshold:
            self.forward_velocity_normalized = 0.0
        elif threshold <= abs(self.forward_velocity) < velocity_change_limit:
            self.forward_velocity_normalized = normal_velocity if self.forward_velocity > 0 else -normal_velocity
        else:
            self.forward_velocity_normalized = high_velocity if self.forward_velocity > 0 else -high_velocity
        
    def compute_angular_velocity(self):
        """
        Computes the angular velocity in radians per second based on the change in angle.
        Updates the self.angular_velocity property.
        """
        if self.previous_yaw is None:
            self.angular_velocity = 0.0
            return

        # Calculate the difference
        delta_angle = self.yaw - self.previous_yaw
        # Normalize the angle difference to the range [-180, 180]
        delta_angle = (delta_angle + 180) % 360 - 180
        # Compute angular velocity
        angular_velocity_deg = delta_angle / self.current_delta_time
        # Convert to radians
        self.angular_velocity = math.radians(angular_velocity_deg)

    def clamp_angular_velocity(self, max_angular_velocity=1.5):
        """
        Clamps the angular velocity to a specified maximum value.
        Updates the self.angular_velocity_clamped property.
        """
        if abs(self.angular_velocity) > max_angular_velocity:
            self.angular_velocity_clamped = math.copysign(max_angular_velocity, self.angular_velocity)
        else:
            self.angular_velocity_clamped = self.angular_velocity

    def process_internal_values(self):
        """
        Processes the values obtained from the KATVR device.
        This method is called in the main loop to update the KATVR properties.
        """
        
        # Offset the yaw to coincide with the KAT Device Simulator
        self.apply_special_yaw_correction()

        # Compute the angular velocity
        self.compute_angular_velocity()

        # Normalize forward velocity
        self.compute_forward_velocity_normalized()

        # Clamp angular velocity
        self.clamp_angular_velocity()

        # Update previous yaw
        self.previous_yaw = self.yaw


class KATVRManager:
    """Manages the KATVR device connection and data processing."""
    def __init__(self, zmq_address="tcp://localhost:5555"):
        self.katvr = KATVR()
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.connect(zmq_address)
        self.testing_mode = TESTING_MODE

    def message_handler(self, address, *args):
        if address != "/katvr":
            print(f"Received message from unknown address: {address}")
            return
        if not self.katvr.is_active:
            self.katvr.is_active = True
            print("KATVR connection established.")

        self.katvr.current_delta_time, self.katvr.uncorrected_yaw, self.katvr.forward_velocity = args
        self.katvr.process_internal_values()

        if not self.testing_mode:
            self.send_to_oculus_client()

    def send_to_oculus_client(self):
        data = {
            "yaw": round(self.katvr.yaw, 2),
            "velocity": round(self.katvr.forward_velocity_normalized, 2)
        }
        payload = json.dumps(data).encode("utf-8")
        self.socket.send_string("from_katvr", zmq.SNDMORE)
        self.socket.send(payload)

    def monitor_loop(self, update_interval=0.05):
        """Continuously monitors and prints KATVR sensor data."""
        while True:
            if not self.katvr.is_active:
                time.sleep(0.1)
                continue

            k = self.katvr
            print(
                f"YAW: {k.yaw:.2f}° | "
                f"Forward Velocity: {k.forward_velocity:.2f} m/s ({k.forward_velocity_normalized:.2f}) | "
                f"Angular Velocity: {k.angular_velocity:.2f} rad/s ({k.angular_velocity_clamped:.2f})"
                f"                  ",
                end="\r"
            )
            time.sleep(update_interval)

    # ================ TESTING METHODS ================

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
    
    def run_interactive_tests(self):
        """Runs the interactive test sequence using KATVR data."""
        print("\n=== KATVR SENSOR TEST SEQUENCE ===\n")

        input("TEST 1. Prepare to turn in place. Press ENTER when ready...")
        self.run_test("turning_in_place", 30, 10)

        input("TEST 2. Prepare to walk forward. Press ENTER when ready...")
        self.run_test("walking_forward", 30, 10)

        print("\n=== TEST SEQUENCE ENDED ===")

    

# --- OSC SERVER ---
def start_osc_server(callback):
    # Local Python server details
    IP, PORT = "127.0.0.1", 8002
    disp = dispatcher.Dispatcher()
    disp.map("/*", callback)
    server = osc_server.ThreadingOSCUDPServer((IP, PORT), disp)
    print(f"Listening for OSC messages on {IP}:{PORT}")
    server.serve_forever()


# ================ MAIN ================
if __name__ == "__main__":
    katvr_manager = KATVRManager()

    threading.Thread(
        target=start_osc_server,
        args=(katvr_manager.message_handler,),
        daemon=True
    ).start()

    try:
        if katvr_manager.testing_mode:
            katvr_manager.run_interactive_tests()
        else:
            katvr_manager.monitor_loop()
    except KeyboardInterrupt:
        print("\nExiting...")