import zmq
import json
import time
import struct
import math


class InputProcessor:
    def __init__(self):
        self.last_inputs = None
        self.inputs: dict = {}
        self.standard_inputs: dict = {}
        self.alternative_inputs: dict = {}
        self.last_oculus_message_time = None
        # KATVR integration properties
        self.katvr: KATVRCalibration = KATVRCalibration()
        self.last_katvr_message_time = None
        # HMD yaw calibration for standard inputs
        self.hmd_yaw_offset = 0.0

    def receive_from_zmq(self):
        context = zmq.Context()
        socket = context.socket(zmq.SUB)
        socket.bind("tcp://*:5555")
        # Subscribe to both topics
        socket.setsockopt_string(zmq.SUBSCRIBE, "from_hmd")
        socket.setsockopt_string(zmq.SUBSCRIBE, "from_katvr")

        while True:
            zmq_topic = socket.recv_string()
            message = socket.recv()
            if zmq_topic == "from_hmd":
                self.oculus_data_processor(message)
            if zmq_topic == "from_katvr":
                self.katvr_data_processor(message)

    def oculus_data_processor(self, message):
        self.last_oculus_message_time = time.time()
        oculus_inputs = list(struct.unpack('18f', message))
        self.inputs['hmd_yaw'] = oculus_inputs[0]  # In radians
        self.inputs['hmd_pitch'] = oculus_inputs[1]  # In radians
        self.inputs['hmd_roll'] = oculus_inputs[2]  # In radians
        self.inputs['left_joystick_x'] = oculus_inputs[3]  # Left Controller X [-1,1]
        self.inputs['left_joystick_y'] = oculus_inputs[4]  # Left Controller Y [-1,1]
        self.inputs['right_joystick_x'] = oculus_inputs[5] # Right Controller X [-1,1]
        self.inputs['right_joystick_y'] = oculus_inputs[6] # Right Controller Y [-1,1]
        self.inputs['button_a'] = oculus_inputs[7]  # Button A (0-1)
        self.inputs['button_b'] = oculus_inputs[8]  # Button B (0-1)
        self.inputs['button_x'] = oculus_inputs[9]  # Button X (0-1)
        self.inputs['button_y'] = oculus_inputs[10]  # Button Y (0-1)
        self.inputs['button_lt'] = oculus_inputs[11]  # Left Thumbstick (0-1)
        self.inputs['button_rt'] = oculus_inputs[12]  # Right Thumbstick (0-1)
        self.inputs['left_trigger'] = oculus_inputs[13]  # Left Index Trigger [0,1]
        self.inputs['right_trigger'] = oculus_inputs[14]  # Right Index Trigger [0,1]
        self.inputs['left_grip'] = oculus_inputs[15]  # Left Grip [0,1]
        self.inputs['right_grip'] = oculus_inputs[16]  # Right Grip [0,1]
        self.inputs['hmd_height'] = oculus_inputs[17]
        self.check_hmd_yaw_calibration()
        self.create_standard_inputs()

    def check_hmd_yaw_calibration(self):
        """Check for button_rt press to trigger HMD yaw calibration for standard inputs"""
        if self.inputs['button_rt'] > 0.5:
            # Calibrate by setting the current HMD yaw as the new zero point
            self.hmd_yaw_offset = self.inputs['hmd_yaw']
            print(f"HMD Yaw calibrated! New offset: {self.hmd_yaw_offset:.3f} radians")

    def create_standard_inputs(self):
        # Apply calibration to HMD yaw for standard inputs
        calibrated_hmd_yaw = self.inputs['hmd_yaw'] - self.hmd_yaw_offset
        calibrated_hmd_yaw = (calibrated_hmd_yaw + math.pi) % (math.pi*2) - math.pi
        
        self.standard_inputs = {
            'hmd_yaw': calibrated_hmd_yaw,  # Calibrated HMD yaw in radians
            'hmd_pitch': self.inputs['hmd_pitch'],  # In radians
            'hmd_roll': self.inputs['hmd_roll'],  # In radians
            'move_forward': self.inputs['left_joystick_y'],
            'move_lateral': self.inputs['left_joystick_x'],
            'rotate': self.inputs['right_joystick_x'],
            'stand': self.inputs['button_a'],  # Command: Stand/Sit
            'sit': self.inputs['button_b'],  # Command: Sit
        }

    def katvr_data_processor(self, message):
        self.last_katvr_message_time = time.time()
        katvr_data = json.loads(message.decode('utf-8'))
        if self.inputs:
            self.inputs['katvr_yaw'] = katvr_data["yaw"]  # In degrees
            self.inputs['katvr_forward_velocity'] = katvr_data["forward_velocity"]  # Forward velocity in m/s
            self.inputs['katvr_lateral_velocity'] = katvr_data["lateral_velocity"]  # Lateral velocity in m/s
            self.create_alternative_inputs()

    def is_katvr_active(self):
        if not self.last_katvr_message_time:
            return False  # KATVR is not active if no message has been received yet
        return time.time() - self.last_katvr_message_time < 10  # Active if last message was received within 10 seconds
    
    def is_oculus_active(self):
        if not self.last_oculus_message_time:
            return False
        return time.time() - self.last_oculus_message_time < 5  # Active if last message was received within 5 seconds

    def create_alternative_inputs(self):
        self.katvr.update_values(self.inputs)
        if self.inputs['button_rt'] > 0.5:
            self.katvr.requires_hmd_calibration = True

        self.alternative_inputs = {
            'hmd_relative_yaw': self.katvr.get_hmd_relative_angle(),  # Relative HDM Yaw (radians)
            'hmd_pitch': self.inputs['hmd_pitch'],  # HDM Pitch (radians)
            'hmd_roll': self.inputs['hmd_roll'],  # HDM Roll (radians)
            'katvr_yaw': self.katvr.yaw,  # KATVR Yaw (degrees)
            'katvr_forward_velocity': self.inputs['katvr_forward_velocity'],  # KATVR Forward Velocity (m/s)
            'katvr_lateral_velocity': self.inputs['katvr_lateral_velocity'],  # KATVR Lateral Velocity (m/s)
            'stand': self.inputs['button_a'],  # Command: Stand
            'sit': self.inputs['button_b'],  # Command: Sit
            'move_forward': self.inputs['right_joystick_y'],  # Move Forward
            'move_lateral': self.inputs['right_joystick_x'],  # Move Lateral
            'rotate': self.inputs['left_joystick_x'],  # Rotate
            'speed_lock': (1.0 if self.inputs['right_trigger'] > 0.5 else 0.0),  # Speed Lock (1.0 if pressed, else 0.0)
            'rotation_lock': (1.0 if self.inputs['right_grip'] > 0.5 else 0.0),  # Rotation Lock (1.0 if pressed, else 0.0)
            'hmd_height': self.inputs['hmd_height'],  # HDM Height
        }



class KATVRCalibration:
    """
    Handles the calibration between the KATVR device and the head-mounted display (HMD)
    """
    def __init__(self):
        self.hmd_yaw = None
        self.yaw = None
        self.requires_hmd_calibration = True
        self.offset = 0

    # --- PRIVATE METHODS ---
    def normalize_angle(self, angle):
        return ((angle + 180) % 360) - 180

    def calibrate_with_hmd(self):
        if self.requires_hmd_calibration:
            self.offset = self.normalize_angle(self.hmd_yaw - self.yaw)
            self.requires_hmd_calibration = False
        
    # --- PUBLIC METHODS ---
    def update_values(self, inputs):
        self.yaw = inputs['katvr_yaw']  # In degrees
        self.hmd_yaw = self.normalize_angle(math.degrees(inputs['hmd_yaw'])) 

    def get_hmd_relative_angle(self):
        """
        Get the head-relative angle between the head-mounted display (HMD) and the KATVR device.
        The result is in radians.
        """
        self.calibrate_with_hmd()
        relative_angle = self.normalize_angle(self.hmd_yaw - self.yaw - self.offset)
        return math.radians(relative_angle)  # Convert to radians