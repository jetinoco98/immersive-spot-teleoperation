import zmq
import struct

def receive_from_zmq():
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect("tcp://localhost:5555")

    socket.setsockopt_string(zmq.SUBSCRIBE, "from_cpp")

    print("Waiting for messages...")

    while True:
        topic = socket.recv_string()  # Receive topic
        data = socket.recv()          # Receive one float in binary

        # Unpack a single float
        value = struct.unpack('f', data)[0]

        print(f"Received from C++: {value}")

# Run the subscriber
receive_from_zmq()