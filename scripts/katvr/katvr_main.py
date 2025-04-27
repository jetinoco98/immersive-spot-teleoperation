from scripts.katvr.katvr_osc_sockets import send_osc_message, start_osc_server
import threading
import time
import math
import zmq
import json

# --- GLOBAL CONSTANTS ---
CONTROL_TYPE = "open"


# --- HELPER FUNCTIONS ---
def shift_angle(angle, offset):
    shifted = angle - offset
    # Wrap back to [-180, 180]
    if shifted > 180:
        shifted -= 360
    elif shifted < -180:
        shifted += 360
    return shifted

def speed_control(speed, low_range, med_range):
    abs_speed = abs(speed)
    if low_range <= abs_speed < med_range:
        return 0.5 if speed >= 0 else -0.5
    elif abs_speed >= med_range:
        return 1.0 if speed >= 0 else -1.0
    return 0.0


# --- CLASS DEFINITIONS ---
class KATVR:
    def __init__(self, use_spot_simulation=False):
        self.yaw = 0
        self.move_speed = 0
        self.frequency = 0
        self.first_connection = False
        self.requires_hdm_calibration = True
        self.use_spot_simulation = use_spot_simulation

    def __str__(self):
        return f"yaw: {self.yaw}, move_speed: {self.move_speed}"


class Control:
    def __init__(self, turn=0, move=0):
        self.turn = turn
        self.move = move

    def __str__(self):
        return f"turn: {self.turn}, move: {self.move}"


class TurnOpenLoopController:
    DEG_TO_RAD = math.pi / 180
    RAD_TO_DEG = 180 / math.pi

    def __init__(self, smoothing_factor=0.2, spot_max_angular_velocity=1.5):
        self.spot_max_angular_velocity = spot_max_angular_velocity
        self.smoothing_factor = smoothing_factor
        self.last_yaw = None
        self.angular_velocity_smoothed = 0

    def unwrap_angle(self, current, previous):
        delta = current - previous
        delta = (delta + 180) % 360 - 180 
        return delta

    def process(self, delta_time, yaw):
        if self.last_yaw is None:
            self.last_yaw = yaw
            return [0, 0, 0]

        delta_yaw = self.unwrap_angle(yaw, self.last_yaw)
        raw_angular_velocity = delta_yaw / delta_time
        self.last_yaw = yaw

        # Apply smoothing
        self.angular_velocity_smoothed = (
            self.smoothing_factor * raw_angular_velocity +
            (1 - self.smoothing_factor) * self.angular_velocity_smoothed
        )

        # Normalize control signal
        control_signal = self.angular_velocity_smoothed / (self.RAD_TO_DEG * self.spot_max_angular_velocity)
        control_signal = max(-1, min(1, control_signal))
        if abs(control_signal) < 0.1:
            control_signal = 0

        angular_velocity_smoothed_rad = self.angular_velocity_smoothed * self.DEG_TO_RAD

        return [control_signal, raw_angular_velocity, angular_velocity_smoothed_rad]


class TurnClosedLoopController:
    def __init__(self, kp, kd, deadband):
        self.kp = kp
        self.kd = kd
        self.deadband = deadband
        self.previous_error = 0

    def normalize_angle(self, angle):
        return (angle + 180) % 360 - 180

    def process(self, delta_time, kat_yaw, spot_yaw):
        kat_yaw = self.normalize_angle(kat_yaw)
        spot_yaw = self.normalize_angle(spot_yaw)
        error = self.normalize_angle(kat_yaw - spot_yaw)

        if abs(error) < self.deadband:
            return [0, error]

        proportional = self.kp * error
        derivative = self.kd * (error - self.previous_error) / delta_time if delta_time != 0 else 0
        self.previous_error = error

        control_signal = max(-1, min(1, proportional + derivative))

        if abs(control_signal) < 0.1:
            control_signal = 0

        return [control_signal, error]



# --- MESSAGE HANDLING ---
def message_handler(address, *args):
    global katvr, control, spot_yaw
    katvr.frequency += 1
    katvr.first_connection = True

    if address == "/katvr":
        delta_time, yaw, katvr.move_speed = args
        # Offset the yaw to coincide with the KAT Device Simulator
        katvr.yaw = shift_angle(yaw, 90)
        katvr.yaw =  (-katvr.yaw) if abs(katvr.yaw) > 0 else katvr.yaw

    # Only when using simulator and closed loop control
    elif address == "/spot":
        spot_yaw = args[0]
        return

    # Turn control
    if CONTROL_TYPE == "open":
        output_values = turn_open_loop.process(delta_time, katvr.yaw)
        control.turn = output_values[0]
        
    elif CONTROL_TYPE == "closed" and katvr.yaw is not None:
        output_values = turn_closed_loop.process(delta_time, katvr.yaw, spot_yaw)
        control.turn = output_values[0]
    
    # Movement control
    control.move = speed_control(katvr.move_speed, 0.7, 3.0)

    # print("Received message:", address, args)


# --- SEND DATA TO OCULUS CLIENT ---
def send_to_oculus_client():
    global katvr, control

    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.connect("tcp://localhost:5555")

    data = {
        "turn": round(float(control.turn), 2),
        "move": round(float(control.move), 2),
        "yaw": round(float(katvr.yaw), 2),
        "calibration": False
    }

    if katvr.requires_hdm_calibration:
        data["calibration"] = True
        katvr.requires_hdm_calibration = False

    serialized_data = json.dumps(data).encode('utf-8')

    topic = "from_katvr"
    socket.send_string(topic, zmq.SNDMORE)
    socket.send(serialized_data)
        

# --- SENDER LOGIC ---
def share_katvr_data():
    global katvr, control
    while True:
        if not katvr.first_connection:
            time.sleep(0.5)
            continue

        if katvr.use_spot_simulation:
            send_osc_message([float(control.turn), float(control.move)])
        else:
            send_to_oculus_client()
        # Set a frequency of 20Hz
        time.sleep(0.05)


# --- OPTIONAL: PRINT KATVR FREQUENCY ---
def print_katvr_frequency():
    global katvr
    while True:
        print(katvr.frequency) if katvr.frequency > 0 else None
        katvr.frequency = 0
        # Set a frequency of 1Hz
        time.sleep(1)


# --- MAIN ---
katvr = KATVR(use_spot_simulation=False)
control = Control()
turn_open_loop = TurnOpenLoopController()
turn_closed_loop = TurnClosedLoopController(kp=0.2, kd=0.0, deadband=1.0)
spot_yaw = 0

# Start threads
threading.Thread(target=start_osc_server, args=(message_handler,), daemon=True).start()
threading.Thread(target=share_katvr_data, daemon=True).start()

# For testing purposes
# if there is lag in Unreal Engine, the frequency will be lower than 60.
# threading.Thread(target=print_katvr_frequency, daemon=True).start()

while True:
    try:
        # Calibrate (KATVR - HDM) with command "c"
        command = input("Enter a command: ")
        if command == "c":
            katvr.requires_hdm_calibration = True
    except KeyboardInterrupt:
        print("\nExiting...")
        exit()
    