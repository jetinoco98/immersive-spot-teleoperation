#!/usr/bin/env python

import argparse
import paho.mqtt.client as mqtt
import time
import threading
import json
import os
from input_processor import InputProcessor


class OculusClient:
    def __init__(self, broker_address):
        self.broker_address = broker_address
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.connect(host=self.broker_address, port=1883)
        self.client.loop_start()
    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code " + str(rc))


def main(broker_address):
    # Initialize variables
    # oculus = OculusClient(broker_address)
    iproc = InputProcessor()

    # Start threads
    threading.Thread(target=iproc.receive_from_zmq, daemon=True).start() # Start the ZMQ receiver thread

    try:
        # Temporary: PID config
        last_sent_time = time.time()
        script_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(script_dir, "spot_pid.json")

        while True:
            # Wait for inputs to be received
            if not iproc.inputs:
                time.sleep(0.5)
                continue
            else:
                if iproc.is_katvr_active() and iproc.alternative_inputs:
                    print('\r\033[K' + "Alternative Inputs: " + ", ".join(f"{k}: {v:.2f}" for k, v in iproc.alternative_inputs.items()))
                    payload = json.dumps(iproc.alternative_inputs)
                    # oculus.client.publish(topic="spot/inputs_with_katvr", payload=payload)
                elif iproc.standard_inputs:
                    print('\r\033[K' + "Standard Inputs: " + ", ".join(f"{k}: {v:.2f}" for k, v in iproc.standard_inputs.items()))
                    payload = json.dumps(iproc.standard_inputs)
                    # oculus.client.publish(topic="spot/inputs", payload=payload)

            # Temporary: Send PID config periodically
            now = time.time()
            if now - last_sent_time > 1:  # Send data every second
                with open(config_path, "r") as f:
                    config = json.load(f)
                config_out = {
                    "kp": config.get("Kp", 1.2),
                    "kd": config.get("Kd", 0.05),
                    "dead_zone_degrees": config.get("DEAD_ZONE_DEGREES", 2.0),
                    "max_v_rot_rad_s": config.get("MAX_V_ROT_RAD_S", 1.5)
                }
                payload = json.dumps(config_out)
                # oculus.client.publish(topic="spot/config", payload=payload)
                last_sent_time = now

            # Frequency of 20Hz
            time.sleep(0.05)  

    except KeyboardInterrupt:
        pass

    finally:
        # oculus.client.loop_stop()
        # oculus.client.disconnect()
        print("Oculus client disconnected.")


# ===== MAIN SCRIPT =====
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Send control data to the MQTT server")
    parser.add_argument(
        'ip_address',
        type=str,
        nargs='?',
        default='100.119.186.122',  
        help='The IP address of the MQTT server'
    )
    args = parser.parse_args()
    main(args.ip_address)
