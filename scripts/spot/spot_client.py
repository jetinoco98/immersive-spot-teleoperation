#!/usr/bin/env python

import argparse
import cv2
import json
import paho.mqtt.client as mqtt
import threading
import time
from spot_interface import SpotInterface
from zed_interface import ZEDInterface
from input_controller import Controller


class SpotClient:
    def __init__(self, broker_address):
        self.broker_address = broker_address
        self.sub_topic_spot = "spot/inputs"
        self.sub_topic_spot2 = "spot/inputs_with_katvr"  # For KATVR
        self.sub_topic_spot_config = "spot/config"
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
        client.subscribe(self.sub_topic_spot_config)

    
    def process_commands(self, stand, sit):
        if sit == 1.0 and self.robot_stand:
            self.interface.sit()
            self.robot_stand = False
        elif stand == 1.0 and not self.robot_stand:
            self.interface.stand()
            self.robot_stand = True


    def on_message(self, client, userdata, msg):

        # ============================================================
        #            MESSAGES RECEIVED ON STANDARD CONTROLS
        # ============================================================
        
        if msg.topic == self.sub_topic_spot:
            # Decode the message payload
            payload = msg.payload.decode()
            inputs = json.loads(payload)

            if not inputs:
                return

            hmd_inputs = [
                inputs['hmd_yaw'],        # HMD Yaw (radians)
                inputs['hmd_pitch'],      # HMD Pitch (radians)
                inputs['hmd_roll'],       # HMD Roll (radians)
            ]
            touch_inputs = [
                inputs['move_forward'],   # Left Joystick Y [-1,1]: Forward/Backward
                inputs['move_lateral'],   # Left Joystick X [-1,1]: Right/Left
                inputs['rotate'],         # Right Joystick X [-1,1]: Angular Velocity
            ]

            # Update HDM inputs with LQR-optimized controls 
            spot_measures = self.interface.get_body_orientation()
            hmd_controls = self.controller.get_hmd_controls(hmd_inputs, spot_measures)

            # Update touch controls according to the designated thresholds on LQR Controller
            touch_controls = self.controller.get_touch_controls(touch_inputs)

            # Process commands
            stand_command_processed = self.process_commands(
                stand=inputs['stand'],  # Command: Stand
                sit=inputs['sit']       # Command: Sit
            )

            # Set the controls for the robot
            if not stand_command_processed and self.robot_stand:
                self.interface.set_hmd_controls(hmd_controls)
                self.interface.set_touch_controls(touch_controls)
                self.interface.send_velocity_command()
            return

        # ============================================================
        #           MESSAGES RECEIVED WITH KATVR CONTROLS
        # ============================================================

        if msg.topic == self.sub_topic_spot_config:
            payload = msg.payload.decode()
            config = json.loads(payload)

            kp = config.get("kp", 1.2)
            kd = config.get("kd", 0.2)
            dead_zone_degrees = config.get("dead_zone_degrees", 2.0)
            max_v_rot_rad_s = config.get("max_v_rot_rad_s", 1.0)

            self.interface.set_pid_controller(kp, kd, dead_zone_degrees, max_v_rot_rad_s)

        if msg.topic == self.sub_topic_spot2:
            payload = msg.payload.decode()
            inputs = json.loads(payload)

            if not inputs:
                return

            # Update HDM inputs with LQR-optimized controls
            hmd_inputs = [
                inputs['hdm_relative_yaw'],   # Relative HDM Yaw (radians)
                inputs['hdm_pitch'],          # HDM Pitch (radians)
                inputs['hdm_roll'],           # HDM Roll (radians)
            ]
            spot_measures = self.interface.get_body_orientation()
            hmd_controls = self.controller.get_hmd_controls(hmd_inputs, spot_measures)

            # Update touch controls according to the designated thresholds
            touch_inputs_alternative = [
                inputs['move_forward'],  # Right Controller Y (0-1): Forward/Backward
                inputs['move_lateral'],  # Right Controller X (0-1): Right/Left
                0.0,       # Control for angular velocity (not used in this case)
            ]
            touch_controls = self.controller.get_touch_controls(touch_inputs_alternative)
            touch_controls[2] = None    # No angular control for KATVR

            # Process commands
            stand_command_processed = self.process_commands(
                stand=inputs['stand'], 
                sit=inputs['sit']
            )

            # Set the controls for the robot
            if not stand_command_processed and self.robot_stand:
                self.interface.set_hmd_controls(hmd_controls)
                self.interface.set_touch_controls(touch_controls)
                self.interface.set_katvr_command(inputs)


    def control_loop(self):
        try:
            while True:
                # Update the robot's odometry angles at a rate of 10Hz
                self.interface.update_odometry_angles()
                self.interface._is_odometry_updated = True
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            self.interface.shutdown()
            self.client.loop_stop()
            self.client.disconnect()
            print("Spot Client disconnected.")
        

def main(broker_address):
    zed = ZEDInterface()
    threading.Thread(target=zed.start_streaming, args=(broker_address,)).start()
    spot = SpotClient(broker_address)
    threading.Thread(target=spot.control_loop).start()


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
