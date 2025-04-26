from pythonosc.udp_client import SimpleUDPClient
from pythonosc import dispatcher, osc_server

def send_osc_message(values):
    # UE5 OSC server details
    UE5_IP, UE5_PORT = "127.0.0.1", 8001
    client = SimpleUDPClient(UE5_IP, UE5_PORT)
    try:
        client.send_message("/katvr", values)  # Send list as a single message
    except ValueError as e:
        print(f"Error: {e}. Invalid input.")


def start_osc_server(handler_function):
    # Local Python server details
    IP, PORT = "127.0.0.1", 8002
    disp = dispatcher.Dispatcher()
    disp.map("/*", handler_function)
    server = osc_server.ThreadingOSCUDPServer((IP, PORT), disp)
    print(f"Listening for OSC messages on {IP}:{PORT}")
    server.serve_forever()

