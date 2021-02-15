#include "pr_sensors_actuators/simulated/encoders_simulink_socket.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>
#include <string>
#include <vector>
#include <algorithm>
#include <iterator>
#include <cassert>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "pr_msgs/msg/pr_array_h.hpp"

#include "rclcpp/qos.hpp"

using std::placeholders::_1;


namespace pr_sensors_actuators
{
    /**** ENCODERS SIMULINK COMPONENT ****/
    
    EncodersSimulinkSocket::EncodersSimulinkSocket(const rclcpp::NodeOptions & options)
    : Node("position_sensors", options)
    {
        //Parameter declaration
        this->declare_parameter<float>("ts_ms", 10.0);
        this->declare_parameter<std::vector<double>>("initial_position", {0.665559, 0.654974, 0.691026, 0.631511});

        //Read parameters
        this->get_parameter("ts_ms", ts);
        this->get_parameter("initial_position", initial_position);

        //Position publisher
        publisher_ = this->create_publisher<pr_msgs::msg::PRArrayH>(
			"joint_position", 
			1);

        //Create timer
        timer_ = this->create_wall_timer(
            std::chrono::duration<float, std::milli>(ts), 
            std::bind(&EncodersSimulinkSocket::on_timer, this));

        //Create socket file descriptor
        int server_fd; 
        struct sockaddr_in address; 
        int opt = 1; 
        int addrlen = sizeof(address);

        server_fd = socket(AF_INET, SOCK_STREAM, 0);

        setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt));
        
        address.sin_family = AF_INET; 
        address.sin_addr.s_addr = INADDR_ANY; 
        address.sin_port = htons( PORT );

        bind(server_fd, (struct sockaddr *)&address, sizeof(address));

        listen(server_fd, 3);

        new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen);

    }


    void EncodersSimulinkSocket::on_timer() {
        char buffer[127] = {0};

        auto pos_msg = pr_msgs::msg::PRArrayH();

        int valread = read(new_socket, buffer, 127);
		std::vector<float> v;
        std::istringstream iss(buffer);

        std::copy(std::istream_iterator<float>(iss),
                  std::istream_iterator<float>(),
                  std::back_inserter(v));

        for(int i=0; i<4; i++)
            pos_msg.data[i] = v[i];

        pos_msg.current_time = this->get_clock()->now();
        pos_msg.header.stamp = pos_msg.current_time;

        publisher_->publish(pos_msg); 
    }


}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_sensors_actuators::EncodersSimulinkSocket)