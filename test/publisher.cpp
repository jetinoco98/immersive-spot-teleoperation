#include <zmq.hpp>
#include <iostream>
#include <thread>
#include <chrono>
#include <cstring>

int main() {
    // Initial data
    float data[] = { 0.0f, -2.0f, -3.0f, 4.0f, 5.0f, 6.0f };

    zmq::context_t zmq_context(1);
    zmq::socket_t publisher(zmq_context, zmq::socket_type::pub);
    publisher.bind("tcp://*:5555"); 

    std::string topic = "from_cpp";  

    while (true) {
        zmq::message_t topic_msg(topic.data(), topic.size());
        publisher.send(topic_msg, zmq::send_flags::sndmore);

        // Increment first value
        data[0] += 1.0f;

        zmq::message_t data_msg(sizeof(data)); 
        std::memcpy(data_msg.data(), data, sizeof(data));
        publisher.send(data_msg, zmq::send_flags::none);

        printf("Sent data: %.1f, %.1f, %.1f, %.1f, %.1f, %.1f\n", 
               data[0], data[1], data[2], data[3], data[4], data[5]);

        // Send every second
        std::this_thread::sleep_for(std::chrono::seconds(1));  
    }

    return 0;
}