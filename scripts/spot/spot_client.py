#!/usr/bin/env python

import argparse
import cv2
import json
import paho.mqtt.client as mqtt
import threading
import socket
import os
import time
from spot_interface import SpotInterface
from zed_interface import ZEDInterface
from input_controller import Controller


def check_internet_connection():
    try:
        # Attempt to create a socket connection to Google's DNS server
        socket.create_connection(("8.8.8.8", 53), timeout=15)
        return True
    except OSError:
        return False

def reconnect():
    print("reconnecting...")
    os.system("sudo systemctl restart simcom_wwan@wwan0.service")
    time.sleep(5)

class SpotClient:
    def __init__(self, broker_address):
        self.broker_address = broker_address
        self.sub_topic_spot = "spot/inputs"
        self.sub_topic_spot2 = "spot/inputs2"  # For KATVR
        self.sub_topic_arm = "arm/inputs"
        self.sub_topic_rtt = "spot/command"
        self.pub_topic_rtt = "spot/response"
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect(self.broker_address, 1883)
        self.client.loop_start()
        self.robot_stand = False
        self.controller = Controller()
        self.interface = SpotInterface()
        

    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code " + str(rc))
        client.subscribe(self.sub_topic_spot)
        client.subscribe(self.sub_topic_spot2)  # For KATVR
        client.subscribe(self.sub_topic_rtt)

    
    def process_stand_command(self, command):
        """
        Processes a stand or sit command for the robot.

        Args:
            command (float): The command to process. 
                             Use 1.0 to make the robot stand and 0.0 to make it sit.

        Returns:
            bool: True if an interface call to stand or sit is made, otherwise False.
        """
        if command == 1.0 and not self.robot_stand:
            self.interface.stand()
            self.robot_stand = True
            return True
        elif command == 0.0 and self.robot_stand:
            self.interface.sit()
            self.robot_stand = False
            return True
        return False


    def on_message(self, client, userdata, msg):

        if msg.topic == self.sub_topic_spot:
            # Decode the message payload
            payload = msg.payload.decode()
            inputs = json.loads(payload)

            if not inputs:
                print("Received empty inputs from spot/inputs.")
                return

            """
            The inputs list from spot/inputs is expected to be in the following format:
            0. HDM Yaw (radians)
            1. HDM Pitch (radians)
            2. HDM Roll (radians)
            3. Left Controller Y (0-1)
            4. Left Controller X (0-1)
            5. Right Controller X (0-1)
            6. Command: Stand/Sit
            """
            
            hmd_inputs = inputs[0:3]
            touch_inputs = inputs[3:6]
            
            # Update HDM inputs with LQR-optimized controls
            spot_measures = self.interface.get_body_orientation()
            hmd_controls = self.controller.get_hmd_controls(hmd_inputs, spot_measures)

            # Update touch controls according to the designated thresholds on LQR Controller
            touch_controls = self.controller.get_touch_controls(touch_inputs)

            # Process the stand command
            stand_command = inputs[6]
            stand_command_processed = self.process_stand_command(stand_command)
            
            # Set the controls for the robot
            if not stand_command_processed and self.robot_stand:
                self.interface.set_hmd_controls(hmd_controls)
                self.interface.set_touch_controls(touch_controls)

        # -- Message received when KATVR is being used --
        elif msg.topic == self.sub_topic_spot2:
            payload = msg.payload.decode()
            inputs = json.loads(payload)

            if not inputs:
                print("Received empty inputs from spot/inputs2.")
                return

            """
            The inputs list from spot/inputs2 is expected to be in the following format:
            0. Relative HDM Yaw (radians)
            1. HDM Pitch (radians)
            2. HDM Roll (radians)
            3. KATVR Yaw (degrees)
            4. KATVR Forward Velocity (m/s)
            5. Command: Stand/Sit
            6. Command: Alignment
            """
            
            hmd_inputs = inputs[0:3]
            # Update HDM inputs with LQR-optimized controls
            spot_measures = self.interface.get_body_orientation()
            hmd_controls = self.controller.get_hmd_controls(hmd_inputs, spot_measures)

            # Process the stand command
            stand_command = inputs[5]
            stand_command_processed = self.process_stand_command(stand_command)

            katvr_inputs = [
                inputs[3],  # KATVR Yaw (degrees)
                inputs[4],  # KATVR Forward Velocity (m/s)
                inputs[6]   # Command: Alignment
            ]

            # Set the controls for the robot
            if not stand_command_processed and self.robot_stand:
                self.interface.set_hmd_controls(hmd_controls)
                self.interface.set_katvr_inputs(katvr_inputs)


        elif msg.topic == self.sub_topic_rtt:
            command = msg.payload.decode()
            client.publish(self.pub_topic_rtt, payload=command)


    def control_loop(self):
        try:
            while True:
                time.sleep(0.05)
                
        except KeyboardInterrupt:
            pass

        finally:
            self.interface.shutdown()
            self.client.loop_stop()
            self.client.disconnect()
            print("Spot Client disconnected.")


def stream_loop(broker_address):
    zed = ZEDInterface()
    try:
        #key = ' '
        while True:
            pipeline = 'appsrc ! videoconvert ! videoscale ! video/x-raw,format=YUY2,width=1280,height=360,framerate=60/1 ! nvvidconv ! \
                            nvv4l2h264enc bitrate=3000000 ! video/x-h264, \
                            stream-format=byte-stream ! \
                            rtspclientsink protocols=udp location=rtsp://%s:8554/spot-stream' % broker_address
                            #whipsink url=http://%s:8889/spot-stream/publish' % broker_address

            image_size = (1280, 360)

            out_send = cv2.VideoWriter(pipeline, cv2.CAP_GSTREAMER, 0, 60, image_size,  True)
            if not out_send.isOpened():
                print('VideoWriter not opened')
                exit(0)
            # output  rtsp info
            print("\n *** Launched RTSP Streaming at rtsp://%s:8554/spot-stream ***\n\n" %broker_address)
            connected = check_internet_connection()
            while connected:
                image = zed.get_image()
                out_send.write(image)
                #key = cv2.waitKey(10)
                connected = check_internet_connection()
            print("disconnected...")
            while not connected:
                reconnect()
                connected = check_internet_connection()

    except KeyboardInterrupt:
        pass

    finally:
        zed.shutdown()
        
        
def main(broker_address):
    connected = check_internet_connection()

    while not connected:
        reconnect()
        connected = check_internet_connection()

    stream_thread = threading.Thread(target=stream_loop, args=(broker_address,))
    stream_thread.start()
    spot = SpotClient(broker_address)
    control_thread = threading.Thread(target=spot.control_loop)
    control_thread.start()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Send video stream to RTSP server")
    parser.add_argument(
        'ip_address',
        type=str,
        nargs='?',
        default='48.209.18.239',  
        help='The IP address of the RTSP server'
    )

    args = parser.parse_args()
    main(args.ip_address)
