from osc_server import start_osc_server
import threading
import time
import math
import zmq
import json


# --- CONSTANTS ---
FORWARD_VELOCITY_THRESHOLD = 1.5        # The minimum velocity obtained from KATVR to be considered as forward movement
FORWARD_VELOCITY_CHANGE_LIMIT = 2.5     # The velocity limit above which the forward velocity is considered high
FORWARD_VELOCITY_NORMAL = 0.25          # NORMAL SPEED
FORWARD_VELOCITY_HIGH = 0.5             # HIGH SPEED


# --- CLASSES ---
class KATVR:
    def __init__(self):
        # KATVR device properties
        self.previous_yaw = None
        self.yaw = 0
        self.current_delta_time = None
        self.forward_velocity = 0
        self.forward_velocity_normalized = 0
        self.angular_velocity = 0
        self.angular_velocity_clamped = 0  
        # Additional properties
        self.is_active = False

    # --- INTERNAL METHODS ---
    def apply_special_yaw_correction(self):
        """
        Corrects the yaw value obtained from KATVR by applying an offset of 90°.
        Updates the self.yaw property.
        """
        offset = 90
        self.yaw = (self.yaw - offset + 180) % 360 - 180

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
    

# --- MESSAGE HANDLING AND PROCESSING ---
def send_to_oculus_client():
    data = {
        "yaw": round(float(katvr.yaw), 2),
        "vel": round(float(katvr.forward_velocity_normalized), 2),
        "ang_vel": round(float(katvr.angular_velocity_clamped), 2),
    }

    serialized_data = json.dumps(data).encode('utf-8')
    topic = "from_katvr"
    socket.send_string(topic, zmq.SNDMORE)
    socket.send(serialized_data)


def message_handler(address, *args):
    if address != "/katvr":
        print(f"Received message from unknown address: {address}")
        return

    if not katvr.is_active:
        katvr.is_active = True
        print("KATVR connection established.")

    katvr.current_delta_time, katvr.yaw, katvr.forward_velocity = args
    katvr.process_internal_values()

    # Send the values to the Oculus client
    send_to_oculus_client()


# --- MAIN ---

#  ZMQ Init 
context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.connect("tcp://localhost:5555")

# Initialize KATVR instance to be used globally
katvr = KATVR() 

# Start the OSC UDP Socket server in a separate thread
threading.Thread(target=start_osc_server, args=(message_handler,), daemon=True).start()

while True:
    try:
        if katvr.is_active:
            print(
                f"YAW: {katvr.yaw:.2f}° | "
                f"Forward Velocity: {katvr.forward_velocity:.2f} m/s ({katvr.forward_velocity_normalized:.2f}) | "
                f"Angular Velocity: {katvr.angular_velocity:.2f} rad/s ({katvr.angular_velocity_clamped:.2f})",
                end="        \r"
            )
        time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nExiting...")
        break
