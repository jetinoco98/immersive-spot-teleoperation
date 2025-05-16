from pythonosc import dispatcher, osc_server

def start_osc_server(handler_function):
    # Local Python server details
    IP, PORT = "127.0.0.1", 8002
    disp = dispatcher.Dispatcher()
    disp.map("/*", handler_function)
    server = osc_server.ThreadingOSCUDPServer((IP, PORT), disp)
    print(f"Listening for OSC messages on {IP}:{PORT}")
    server.serve_forever()

