#include "pr_sensors_actuators/simulated/motors_simulink_socket.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

using std::placeholders::_1;


namespace pr_sensors_actuators
{
    /**** MOTOR SIMULINK COMPONENT ****/
    
    MotorsSimulinkSocket::MotorsSimulinkSocket(const rclcpp::NodeOptions & options)
    : Node("motors", options)
    {
        //Parameter declaration
        this->declare_parameter<double>("max_v", 3.0);
        this->declare_parameter<std::vector<double>>("vp_conversion", {28.4628, 28.4628, 28.4628, 246.6779});

        //Read parameters
        this->get_parameter("vp_conversion", vp_conversion);
        this->get_parameter("max_v", max_v);


        //Control action sub
        subscription_ = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "control_action",
            10,
            std::bind(&MotorsSimulinkSocket::topic_callback,this,_1));

        RCLCPP_INFO(this->get_logger(), "Motors initialized");

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

    MotorsSimulinkSocket::~MotorsSimulinkSocket()
    {
        
    }

    void MotorsSimulinkSocket::topic_callback(const pr_msgs::msg::PRArrayH::SharedPtr control_action_msg) 
    {
        
        volts[0] = control_action_msg->data[0]/vp_conversion[0];
        volts[1] = control_action_msg->data[1]/vp_conversion[1];
        volts[2] = control_action_msg->data[2]/vp_conversion[2];
        volts[3] = control_action_msg->data[3]/vp_conversion[3];

        sat_ca(volts[0], max_v);
        sat_ca(volts[1], max_v);
        sat_ca(volts[2], max_v);
        sat_ca(volts[3], max_v);

        std::string v_msg;

        v_msg = std::to_string(volts[0]) + " " + 
                std::to_string(volts[1]) + " " + 
                std::to_string(volts[2]) + " " + 
                std::to_string(volts[3]);

        send(new_socket, v_msg.c_str(), strlen(v_msg.c_str()), 0);

    }

    void MotorsSimulinkSocket::sat_ca(double &control_action, const double &sat)
    {
	     if(control_action > sat) control_action = sat;
	     if(control_action < -sat) control_action = -sat;
    }

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_sensors_actuators::MotorsSimulinkSocket)