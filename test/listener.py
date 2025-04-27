import zmq
import struct

def receive_from_zmq():
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect("tcp://localhost:5555")

    socket.setsockopt_string(zmq.SUBSCRIBE, "from_cpp")

    print("Waiting for messages...")

    while True:
        # Receive the topic AND data
        topic = socket.recv_string()  # This should be "from_cpp"
        data = socket.recv()  # this is binary data

        # Convert the binary data into a list of 6 floats
        floats = struct.unpack('6f', data)  

        print(f"Received from C++: {floats}")

# Run the subscriber
receive_from_zmq()