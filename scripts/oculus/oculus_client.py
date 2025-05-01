#!/usr/bin/env python

import argparse
import paho.mqtt.client as mqtt
import json
import time

# -- KATVR: New imports --
import zmq
import json
import struct
import threading
from oculus_katvr_calibration import KATVRInputs, HDMCalibrator, update_inputs


# -- New global variables --
katvr = KATVRInputs()
hdm_calibrator = HDMCalibrator()
inputs = [0,0,0,0,0,0]


# -- New functions for receiving data --
def katvr_data_processor(message):
    global katvr

    data = json.loads(message.decode('utf-8'))

    katvr.is_active = True
    katvr.turn = data["turn"]
    katvr.move = data["move"]
    katvr.yaw = data["yaw"]

    if data["calibration"] == True:
        katvr.requires_calibration = True


def hdm_data_processor(message):
    global inputs
    try:
        inputs = list(struct.unpack('7f', message))
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


# -- OCULUS CLIENT ORIGINAL CODE --
port = 1883
topic = "oculus/inputs"

class OculusClient:
    def __init__(self, broker_address):
        self.broker_address = broker_address
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.connect(self.broker_address, port)
        self.client.loop_start()
    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code " + str(rc))


# -- MAIN FUNCTION --
def main(broker_address):
    oculus = OculusClient(broker_address)

    # Listen to incoming messages
    threading.Thread(target=receive_from_zmq, daemon=True).start()

    try:
        while True:
            # Inputs from HDM
            final_inputs = inputs

            # Update to KATVR inputs when active
            if katvr.is_active:
                final_inputs = update_inputs(inputs, katvr, hdm_calibrator)

            print("Inputs: ", final_inputs)
                
            # Publish the inputs through MQTT
            payload = json.dumps(final_inputs)
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
    parser = argparse.ArgumentParser(description="Send video stream to RTSP server")
    parser.add_argument(
        'ip_address',
        type=str,
        nargs='?',
        default='34.16.188.15',  
        help='The IP address of the RTSP server'
    )
    args = parser.parse_args()
    main(args.ip_address)
