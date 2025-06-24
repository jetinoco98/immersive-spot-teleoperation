import zmq
import json
import time
import struct
from katvr_calibration import KATVRCalibration

class InputProcessor:
    def __init__(self):
        self.last_inputs = None
        self.inputs: dict = {}
        self.standard_inputs: dict = {}
        self.alternative_inputs: dict = {}
        # KATVR integration properties
        self.katvr: KATVRCalibration = KATVRCalibration()
        self.last_katvr_message_time = None

    def receive_from_zmq(self):
        context = zmq.Context()
        socket = context.socket(zmq.SUB)
        socket.bind("tcp://*:5555")

        # Subscribe to both topics
        socket.setsockopt_string(zmq.SUBSCRIBE, "from_hdm")
        socket.setsockopt_string(zmq.SUBSCRIBE, "from_katvr")

        while True:
            zmq_topic = socket.recv_string()
            message = socket.recv()

            if zmq_topic == "from_hdm":
                self.oculus_data_processor(message)
            if zmq_topic == "from_katvr":
                self.katvr_data_processor(message)

    def oculus_data_processor(self, message):
        oculus_inputs = list(struct.unpack('17f', message))
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
        self.create_standard_inputs()

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
    
    def create_standard_inputs(self):
        self.standard_inputs = {
            'hmd_yaw': self.inputs['hmd_yaw'],  # In radians
            'hmd_pitch': self.inputs['hmd_pitch'],  # In radians
            'hmd_roll': self.inputs['hmd_roll'],  # In radians
            'move_forward': self.inputs['left_joystick_y'],
            'move_lateral': self.inputs['left_joystick_x'],
            'rotate': self.inputs['right_joystick_x'],
            'stand': self.inputs['button_a'],  # Command: Stand/Sit
            'sit': self.inputs['button_b'],  # Command: Sit
        }

    def create_alternative_inputs(self):
        self.katvr.update_values(self.inputs)
        if self.inputs['button_rt'] > 0.5:
            self.katvr.requires_hdm_calibration = True

        self.alternative_inputs = {
            'hdm_relative_yaw': self.katvr.get_hdm_relative_angle(),  # Relative HDM Yaw (radians)
            'hdm_pitch': self.inputs['hmd_pitch'],  # HDM Pitch (radians)
            'hdm_roll': self.inputs['hmd_roll'],  # HDM Roll (radians)
            'katvr_yaw': self.katvr.yaw,  # KATVR Yaw (degrees)
            'katvr_forward_velocity': self.inputs['katvr_forward_velocity'],  # KATVR Forward Velocity (m/s)
            'katvr_lateral_velocity': self.inputs['katvr_lateral_velocity'],  # KATVR Lateral Velocity (m/s)
            'stand': self.inputs['button_a'],  # Command: Stand
            'sit': self.inputs['button_b'],  # Command: Sit
            'move_forward': self.inputs['right_joystick_y'],  # Move Forward
            'move_lateral': self.inputs['right_joystick_x'],  # Move Lateral
            'speed_lock': (1.0 if self.inputs['right_grip'] > 0.5 else 0.0),  # Speed Lock (1.0 if pressed, else 0.0)
            'rotation_lock': (1.0 if self.inputs['right_trigger'] > 0.5 else 0.0),  # Rotation Lock (1.0 if pressed, else 0.0)
        }

