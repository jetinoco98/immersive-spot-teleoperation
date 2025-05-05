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
inputs = [0,0,0,0,0,0,0]


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


# -- MAIN FUNCTION --
def main(broker_address):

    # Listen to incoming messages
    threading.Thread(target=receive_from_zmq, daemon=True).start()

    try:
        while True:
            # Inputs from HDM
            final_inputs = inputs

            # Update to KATVR inputs when active
            if katvr.is_active:
                final_inputs = update_inputs(inputs, katvr, hdm_calibrator)

            print("Inputs:", [f"{x:.2f}" for x in final_inputs])

            # Frequency of 20Hz
            time.sleep(0.05)

    except KeyboardInterrupt:
        pass

    finally:
        print("Oculus client disconnected.")


if __name__ == '__main__':
    main()
