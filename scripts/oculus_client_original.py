#!/usr/bin/env python

import argparse
import struct
import paho.mqtt.client as mqtt
import json
import time

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

def main(broker_address):
    pipe_path = r'\\.\pipe\MyPipe'
    pipe = open(pipe_path, 'rb')
    oculus = OculusClient(broker_address)
    try:
        while True:
            data = pipe.read(24) 
            received_tuple = struct.unpack('6f', data)
            inputs = list(received_tuple)
            payload = json.dumps(inputs)
            oculus.client.publish(topic, payload)
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        pipe.close()
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
