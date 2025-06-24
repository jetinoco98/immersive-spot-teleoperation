#!/usr/bin/env python

import argparse
import paho.mqtt.client as mqtt
import time
import threading
import json
import os
from input_processor import InputProcessor

# ========== Rich Library for Console Output =========
from rich.table import Table
from rich.live import Live
from rich.console import Console

def dict_to_table(title, data_dict: dict):
    table = Table(title=title, expand=False, show_lines=True)
    table.add_column("Key", style="bold cyan", no_wrap=True)
    table.add_column("Value", justify="right", style="green")

    for k, v in data_dict.items():
        table.add_row(k, f"{v:.2f}")

    return table

console = Console()
live = Live(console=console, refresh_per_second=20)
live.start()
# =====================================================


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
    # Initialize variables and threads
    oculus = OculusClient(broker_address)
    input_processor = InputProcessor()
    threading.Thread(target=input_processor.receive_from_zmq, daemon=True).start() # Start the ZMQ receiver thread

    try:
        # Temporary: PID config
        last_sent_time = time.time()
        script_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(script_dir, "spot_pid.json")

        while True:
            # Wait for inputs to be received
            if not input_processor.inputs:
                time.sleep(0.5)
                continue
            
            if input_processor.is_katvr_active() and input_processor.alternative_inputs:
                live.update(dict_to_table("Alternative Inputs", input_processor.alternative_inputs))
                payload = json.dumps(input_processor.alternative_inputs)
                oculus.client.publish(topic="spot/inputs_with_katvr", payload=payload)
            elif input_processor.standard_inputs:
                live.update(dict_to_table("Standard Inputs", input_processor.standard_inputs))
                payload = json.dumps(input_processor.standard_inputs)
                oculus.client.publish(topic="spot/inputs", payload=payload)

            # Temporary: Send PID config periodically
            if input_processor.is_katvr_active():
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
                    oculus.client.publish(topic="spot/config", payload=payload)
                    last_sent_time = now

            # Frequency of 20Hz
            time.sleep(0.05)  

    except KeyboardInterrupt:
        pass

    finally:
        oculus.client.loop_stop()
        oculus.client.disconnect()
        live.stop()
        print("\nOculus client disconnected.")


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
