#!/usr/bin/env python

import argparse
import paho.mqtt.client as mqtt
import time
import threading
import json
import struct
import sys
import zmq
from oculus_katvr_calibration import KATVRCalibration


# -- OCULUS CLIENT --
port = 1883
topic = "spot/inputs"

class OculusClient:
    def __init__(self, broker_address):
        self.broker_address = broker_address
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.connect(self.broker_address, port)
        self.client.loop_start()
    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code " + str(rc))


# -- Global variables --
katvr = KATVRCalibration()
last_katvr_message_time = time.time()
inputs = []

''' 
The inputs list received from the Meta Quest is expected to be in the following format:
0. HDM Yaw (radians)
1. HDM Pitch (radians)
2. HDM Roll (radians)
3. Left Controller Y (0-1)
4. Left Controller X (0-1)
5. Right Controller X (0-1)
6. Right Controller Y (0-1)
7. Command: Stand/Sit, from buttons A or B
8. Command: Calibration, from right index finger trigger -> Exclusive for KATVR logic
9. Command: Alignment, from right middle finger trigger -> Exclusive for KATVR logic
'''


# -- Functions for receiving data --
def katvr_data_processor(message):
    global katvr, last_katvr_message_time
    katvr.is_active = True

    # Update the last received time
    last_katvr_message_time = time.time()

    # Decode the message
    data = json.loads(message.decode('utf-8'))
    katvr.yaw = data["yaw"]
    katvr.forward_velocity = data["vel"]
    katvr.angular_velocity = data["ang_vel"]

def hdm_data_processor(message):
    global inputs, katvr
    try:
        inputs = list(struct.unpack('10f', message))
        # Check for calibration on the index #8 of the inputs list
        if inputs[8] == 1 and katvr.is_active:
            print("Received calibration request from HDM")
            katvr.requires_hdm_calibration = True
    except:
        print("Error in incoming message format")

def receive_from_zmq():
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.bind("tcp://*:5555")

    # Subscribe to both topics
    socket.setsockopt_string(zmq.SUBSCRIBE, "from_hdm")
    socket.setsockopt_string(zmq.SUBSCRIBE, "from_katvr")

    while True:
        topic = socket.recv_string()
        message = socket.recv()

        if topic == "from_hdm":
            hdm_data_processor(message)
        if topic == "from_katvr":
            katvr_data_processor(message)


# -- Additional functions --
def monitor_katvr_activity():
    global katvr, last_katvr_message_time
    while True:
        # Check if more than 20 seconds have passed since the last KATVR message
        if time.time() - last_katvr_message_time > 20:
            katvr.is_active = False
        time.sleep(0.1)  # Check every 100ms


def send_pid_config_periodically(broker_address):
    config_path = "spot_pid_config.json"
    topic = "spot/config"
    client = mqtt.Client()
    client.connect(broker_address, 1883)
    client.loop_start()
    try:
        while True:
            try:
                with open(config_path, "r") as f:
                    config = json.load(f)
                config_out = {
                    "kp": config.get("Kp", 1.2),
                    "kd": config.get("Kd", 0.2),
                    "dead_zone_degrees": config.get("DEAD_ZONE_DEGREES", 2.0),
                    "max_v_rot_rad_s": config.get("MAX_V_ROT_RAD_S", 1.0)
                }
                client.publish(topic, json.dumps(config_out))
            except Exception as e:
                print(f"Error sending PID config: {e}")
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        client.loop_stop()
        client.disconnect()


# -- MAIN FUNCTION --
def main(broker_address):
    oculus = OculusClient(broker_address)

    # Start the ZMQ receiver thread
    threading.Thread(target=receive_from_zmq, daemon=True).start()
    # Start the KATVR activity monitor thread
    threading.Thread(target=monitor_katvr_activity, daemon=True).start()

    try:
        while True:
            # Wait for inputs to be received
            if not inputs:
                time.sleep(0.1)
                continue

            # Use the KATVR related inputs when active
            if katvr.is_active:
                # Now include right controller X and Y in alternative inputs
                inputs_alternative = katvr.create_alternative_inputs(inputs)

                sys.stdout.write('\r\033[K')
                sys.stdout.write("Alternative Inputs: " + str([f"{x:.2f}" for x in inputs_alternative]))
                sys.stdout.flush()

                """
                The inputs_alternative list is expected to be in the following format:
                0. Relative HDM Yaw (radians)
                1. HDM Pitch (radians)
                2. HDM Roll (radians)
                3. KATVR Yaw (degrees)
                4. KATVR Forward Velocity (m/s)
                5. Command: Stand/Sit
                6. Command: Alignment
                7. Right Controller X (0-1)
                8. Right Controller Y (0-1)
                """
                    
                # Publish the inputs through MQTT
                payload = json.dumps(inputs_alternative)
                new_topic = "spot/inputs2"
                oculus.client.publish(new_topic, payload)
            
            else:
                # Get the inputs from index #0 to #6
                inputs_standard = inputs[:7]
                sys.stdout.write('\r\033[K')  # Clear the current line
                sys.stdout.write("Standard Inputs: " + str([f"{x:.2f}" for x in inputs_standard]))
                sys.stdout.flush()

                """
                The inputs_standard list is expected to be in the following format:
                0. HDM Yaw (radians)
                1. HDM Pitch (radians)
                2. HDM Roll (radians)
                3. Left Controller Y (0-1)
                4. Left Controller X (0-1)
                5. Right Controller X (0-1)
                6. Command: Stand/Sit
                """

                # Publish the inputs through MQTT
                payload = json.dumps(inputs_standard)
                oculus.client.publish(topic, payload)

            # Frequency of 20Hz
            time.sleep(0.05)

    except KeyboardInterrupt:
        pass

    finally:
        oculus.client.loop_stop()
        oculus.client.disconnect()
        print("Oculus client disconnected.")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Send control data to the MQTT server")
    parser.add_argument(
        'ip_address',
        type=str,
        nargs='?',
        default='34.16.188.15',  
        help='The IP address of the MQTT server'
    )
    args = parser.parse_args()

    # TEMPORARY: Start periodic PID config sender in a thread
    threading.Thread(target=send_pid_config_periodically, args=(args.ip_address,), daemon=True).start()

    main(args.ip_address)
