#!/usr/bin/env python

import argparse
import paho.mqtt.client as mqtt
import json
import threading
import time
from spot_interface import SpotInterface
from zed_interface import ZEDInterface
from input_controller import Controller


class SpotClient:
    def __init__(self, broker_address):
        self.stop_event = threading.Event()  # Event to signal termination
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
        self.robot = SpotInterface()  
        # Variables to store and control inputs
        self.standard_inputs = None
        self.alternative_inputs = None
        self.standard_inputs_last_message_time = 0
        self.alternative_inputs_last_message_time = 0


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
            config: dict = json.loads(payload)
            # Update the PID controller configuration
            self.robot.update_pid_controller(
                config.get("kp"),
                config.get("kd"),
                config.get("dead_zone_degrees")
            )


    def process_standard_inputs(self):
        # Stand/Sit commands
        if self.standard_inputs['stand']:
            self.robot.stand()
        elif self.standard_inputs['sit']:
            self.robot.sit()

        if not self.robot.is_standing:
            return
        
        # Update HDM inputs with LQR-optimized controls 
        hmd_inputs = [
            self.standard_inputs['hmd_yaw'],        # HMD Yaw (radians)
            self.standard_inputs['hmd_pitch'],      # HMD Pitch (radians)
            self.standard_inputs['hmd_roll'],       # HMD Roll (radians)
        ]
        spot_measures = self.robot.get_body_orientation()
        hmd_controls = self.controller.get_hmd_controls(hmd_inputs, spot_measures)

        # Update touch controls according to the designated thresholds
        touch_inputs = [
            self.standard_inputs['move_forward'],   # Left Joystick Y [-1,1]: Forward/Backward
            self.standard_inputs['move_lateral'],   # Left Joystick X [-1,1]: Right/Left
            self.standard_inputs['rotate'],         # Right Joystick X [-1,1]: Angular Velocity
        ]
        touch_controls = self.controller.get_touch_controls(touch_inputs)

        # Set the controls for the robot
        self.robot.set_hmd_controls(hmd_controls)
        self.robot.set_touch_controls(touch_controls)
        self.robot.send_velocity_command()


    def process_alternative_inputs(self):   
        # Process commands
        if self.alternative_inputs['stand']:
            self.robot.stand()
        elif self.alternative_inputs['sit']:
            self.robot.sit()

        if self.robot.auto_sit_in_katvr(self.alternative_inputs):
            return
        
        self.robot.auto_stand_in_katvr(self.alternative_inputs)

        if not self.robot.is_standing:
            return
        
        # Update HDM inputs with LQR-optimized controls
        hmd_inputs = [
            self.alternative_inputs['hmd_relative_yaw'],   # Relative HDM Yaw (radians)
            self.alternative_inputs['hmd_pitch'],          # HDM Pitch (radians)
            self.alternative_inputs['hmd_roll'],           # HDM Roll (radians)
        ]
        spot_measures = self.robot.get_body_orientation()
        hmd_controls = self.controller.get_hmd_controls(hmd_inputs, spot_measures)

        # Update touch controls according to the designated thresholds
        touch_inputs_alternative = [
            self.alternative_inputs['move_forward'],  # Right Controller Y (0-1): Forward/Backward
            self.alternative_inputs['move_lateral'],  # Right Controller X (0-1): Right/Left
            self.alternative_inputs['rotate'],        # Left Controller X (0-1): Angular Velocity
        ]
        touch_controls = self.controller.get_touch_controls(touch_inputs_alternative)

        # Set the controls for the robot.
        # Note: HDM Controls are set inside KATVR command function
        self.robot.set_touch_controls(touch_controls)
        self.robot.set_katvr_command(self.alternative_inputs, hmd_controls)


    def control_loop(self):
        try:
            while not self.stop_event.is_set():
                t0 = time.time()
                time_difference_std = time.time() - self.standard_inputs_last_message_time
                time_difference_alt = time.time() - self.alternative_inputs_last_message_time

                # Process standard inputs if available
                if self.standard_inputs and (time_difference_std < 1.0):
                    self.process_standard_inputs()
                # Process alternative inputs if available (KATVR)
                elif self.alternative_inputs and (time_difference_alt < 1.0):
                    self.process_alternative_inputs()
                else:
                    # If no inputs are received, or they are too old, set the robot to idle mode
                    if self.robot.is_standing:
                        self.robot.set_idle_mode()

                self.robot.print_spot_motion_status()

                # Maintain a maximum control loop frequency of 20 Hz
                elapsed_time = time.time() - t0
                if elapsed_time < 1/20:
                    time.sleep(1/20 - elapsed_time)

        except KeyboardInterrupt:
            pass

        finally:
            self.robot.shutdown()
            self.client.loop_stop()
            self.client.disconnect()
            print("Spot Client disconnected.")
        


def main(broker_address):
    stop_event = threading.Event()
    # Initialize ZED and Spot clients
    zed = ZEDInterface()
    spot = SpotClient(broker_address)
    # Start ZED streaming and Spot control loop in separate threads
    zed.stop_event = stop_event
    spot.stop_event = stop_event
    # Start threads for ZED streaming and Spot control loop
    zed_thread = threading.Thread(target=zed.start_streaming, args=(broker_address,))
    spot_thread = threading.Thread(target=spot.control_loop)

    print("Starting ZED streaming loop.")
    zed_thread.start()
    time.sleep(5)  # Ensure ZED is ready before starting Spot control

    print("Starting Spot control loop. You can now control the robot!")
    spot_thread.start()

    try:
        # Wait for KeyboardInterrupt to stop the threads gracefully
        while zed_thread.is_alive() or spot_thread.is_alive():
            time.sleep(0.1) # Prevent busy-waiting
            
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt detected. Signaling threads to stop...")
        stop_event.set() # Signal all threads to stop

    finally:
        zed_thread.join(timeout=10) 
        spot_thread.join(timeout=10) 

        if zed_thread.is_alive():
            print("ZED thread did not terminate gracefully.")
        if spot_thread.is_alive():
            print("SPOT thread did not terminate gracefully.")
        print("Main program has finished execution.")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Send video stream to RTSP server")
    parser.add_argument(
        'ip_address',
        type=str,
        nargs='?',
        default='100.119.186.122',
        help='The IP address of the RTSP server'
    )
    args = parser.parse_args()
    main(args.ip_address)
