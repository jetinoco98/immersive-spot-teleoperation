#!/usr/bin/env python

import argparse
import paho.mqtt.client as mqtt
import json
import threading
import time
import sys
from spot_interface import SpotInterface
from zed_interface import ZEDInterface
from input_controller import Controller


class SpotClient:
    def __init__(self, broker_address):
        # MQQT related variables
        self.broker_address = broker_address
        self.sub_topic_spot = "spot/inputs"
        self.sub_topic_spot2 = "spot/inputs_with_katvr"  # For KATVR
        self.sub_topic_spot_config = "spot/config"
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect(self.broker_address, 1883)
        self.client.loop_start()
        self.controller = Controller()   # Controller for HDM & Touch controls
        self.interface = SpotInterface()  
        # Variables for robot state
        self.robot_stand = False
        # Variables to store and control inputs
        self.standard_inputs = None
        self.alternative_inputs = None
        self.standard_inputs_last_message_time = None
        self.alternative_inputs_last_message_time = None


    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code " + str(rc))
        client.subscribe(self.sub_topic_spot)
        client.subscribe(self.sub_topic_spot2)  # For KATVR
        client.subscribe(self.sub_topic_spot_config)


    def on_message(self, client, userdata, msg):
        # === MESSAGES RECEIVED ON STANDARD CONTROLS ===
        if msg.topic == self.sub_topic_spot:
            payload = msg.payload.decode()
            inputs = json.loads(payload)
            if inputs:
                self.standard_inputs = inputs
                self.standard_inputs_last_message_time = time.time()

        # === MESSAGES RECEIVED WITH KATVR CONTROLS ===
        if msg.topic == self.sub_topic_spot2:
            payload = msg.payload.decode()
            inputs = json.loads(payload)
            if inputs:
                self.alternative_inputs = inputs
                self.alternative_inputs_last_message_time = time.time()
        
        # === TEMPORARY: PID CONFIGURATION ===
        if msg.topic == self.sub_topic_spot_config:
            payload = msg.payload.decode()
            config = json.loads(payload)
            # Update the PID controller configuration
            self.interface.update_pid_controller(
                config.get("kp", 1.2),
                config.get("kd", 0.2),
                config.get("dead_zone_degrees", 2.0),
                config.get("max_v_rot_rad_s", 1.0)
            )

        
    def process_commands(self, stand, sit):
        if sit == 1.0 and self.robot_stand:
            self.interface.sit()
            self.robot_stand = False
        elif stand == 1.0 and not self.robot_stand:
            self.interface.stand()
            self.robot_stand = True


    def process_standard_inputs(self):
        # Process commands
        self.process_commands(
            stand=self.standard_inputs['stand'],  # Command: Stand
            sit=self.standard_inputs['sit']       # Command: Sit
        )

        if not self.robot_stand:
            return
        
        # Update HDM inputs with LQR-optimized controls 
        hmd_inputs = [
            self.standard_inputs['hmd_yaw'],        # HMD Yaw (radians)
            self.standard_inputs['hmd_pitch'],      # HMD Pitch (radians)
            self.standard_inputs['hmd_roll'],       # HMD Roll (radians)
        ]
        spot_measures = self.interface.get_body_orientation()
        hmd_controls = self.controller.get_hmd_controls(hmd_inputs, spot_measures)

        # Update touch controls according to the designated thresholds
        touch_inputs = [
            self.standard_inputs['move_forward'],   # Left Joystick Y [-1,1]: Forward/Backward
            self.standard_inputs['move_lateral'],   # Left Joystick X [-1,1]: Right/Left
            self.standard_inputs['rotate'],         # Right Joystick X [-1,1]: Angular Velocity
        ]
        touch_controls = self.controller.get_touch_controls(touch_inputs)

        # Set the controls for the robot
        self.interface.set_hmd_controls(hmd_controls)
        self.interface.set_touch_controls(touch_controls)
        self.interface.send_velocity_command()


    def process_alternative_inputs(self):
        # Process commands
        self.process_commands(
            stand=self.alternative_inputs['stand'], 
            sit=self.alternative_inputs['sit']
        )

        if not self.robot_stand:
            return
        
        # Update HDM inputs with LQR-optimized controls
        hmd_inputs = [
            self.alternative_inputs['hdm_relative_yaw'],   # Relative HDM Yaw (radians)
            self.alternative_inputs['hdm_pitch'],          # HDM Pitch (radians)
            self.alternative_inputs['hdm_roll'],           # HDM Roll (radians)
        ]
        spot_measures = self.interface.get_body_orientation()
        hmd_controls = self.controller.get_hmd_controls(hmd_inputs, spot_measures)

        # Update touch controls according to the designated thresholds
        touch_inputs_alternative = [
            self.alternative_inputs['move_forward'],  # Right Controller Y (0-1): Forward/Backward
            self.alternative_inputs['move_lateral'],  # Right Controller X (0-1): Right/Left
            0.0,       # Control for angular velocity (not used in this case)
        ]
        touch_controls = self.controller.get_touch_controls(touch_inputs_alternative)
        touch_controls[2] = None    # No rotation control for KATVR

        # Set the controls for the robot.
        self.interface.set_hmd_controls(hmd_controls)
        self.interface.set_touch_controls(touch_controls)
        self.interface.set_katvr_command(self.alternative_inputs)


    def control_loop(self):
        try:
            while True:
                t0 = time.time()

                # Process standard inputs if available
                if self.standard_inputs and (time.time() - self.standard_inputs_last_message_time < 0.2):
                    self.process_standard_inputs()

                # Process alternative inputs if available (KATVR)
                elif self.alternative_inputs and (time.time() - self.alternative_inputs_last_message_time < 0.2):
                    self.process_alternative_inputs()

                else:
                    # If no inputs are received, or they are too old, set the robot to idle mode
                    if self.robot_stand:
                        self.interface.set_idle_mode()

                # Sleep to maintain a control loop frequency of 10Hz
                elapsed_time = time.time() - t0
                if elapsed_time < 0.1:
                    time.sleep(0.1 - elapsed_time)
                else:
                    print(f"Warning: Control loop took too long ({elapsed_time:.2f} seconds), skipping sleep.")
                
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
