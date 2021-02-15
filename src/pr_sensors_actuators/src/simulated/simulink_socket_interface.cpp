#include "pr_sensors_actuators/simulated/simulink_socket_interface.hpp"

#include <memory>
#include <chrono>
#include <iostream>
#include <memory>
#include <utility>
#include <string>
#include <algorithm>
#include <iterator>
#include <cassert>
#include <sstream>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace pr_sensors_actuators
{
    /**** Simulink Socket Interface ****/
    
    SimulinkSocketInterface::SimulinkSocketInterface(const rclcpp::NodeOptions & options)
    : Node("simiulink_interface", options)
    {
        //Parameter declaration
        this->declare_parameter<double>("max_v", 3.0);
        this->declare_parameter<std::vector<double>>("vp_conversion", {28.4628, 28.4628, 28.4628, 246.6779});
        this->declare_parameter<int>("port", 5021);

        int port;
        this->get_parameter("vp_conversion", vp_conversion);
        this->get_parameter("max_v", max_v);
        this->get_parameter("port", port);

        cb_group_timer = this->create_callback_group(
            rclcpp::callback_group::CallbackGroupType::Reentrant
        );

        cb_group_sub = this->create_callback_group(
            rclcpp::callback_group::CallbackGroupType::Reentrant
        );

        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = cb_group_sub;

        timer_ = this->create_wall_timer(
            std::chrono::duration<float, std::milli>(10.0),
            std::bind(&SimulinkSocketInterface::on_timer, this),
            cb_group_timer
        );

        publisher_ = this->create_publisher<pr_msgs::msg::PRArrayH>(
            "joint_position",
            1
        );
        
        subscription_ = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "control_action",
            1,
            std::bind(&SimulinkSocketInterface::topic_callback, this, _1),
            sub_opt
        );
        

        //Create socket
        int opt = 1;

        // Creating socket file descriptor 
        if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) 
        { 
            RCLCPP_ERROR(this->get_logger(), "Error creating file descriptor"); 
            exit(EXIT_FAILURE); 
        }

        if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) 
        { 
            RCLCPP_ERROR(this->get_logger(), "Error setting socket options"); 
            exit(EXIT_FAILURE); 
        }

        struct sockaddr_in address;
        int addrlen = sizeof(address);
        address.sin_family = AF_INET; 
        address.sin_addr.s_addr = INADDR_ANY; 
        address.sin_port = htons(port); 

        // Forcefully attaching socket to the port 
        if (bind(server_fd, (struct sockaddr *)&address, sizeof(address))<0) 
        { 
            RCLCPP_ERROR(this->get_logger(), "Error attaching socket to port"); 
            exit(EXIT_FAILURE); 
        } 

        RCLCPP_INFO(this->get_logger(), "Server waiting for client");
        std::cout << "Server waiting for client" << std::endl;
        if (listen(server_fd, 3) < 0) 
        { 
            RCLCPP_ERROR(this->get_logger(), "Error listening"); 
            exit(EXIT_FAILURE); 
        }

        if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen))<0) 
        { 
            RCLCPP_ERROR(this->get_logger(), "Error accepting request");  
            exit(EXIT_FAILURE); 
        }

        rclcpp::Rate rate(1);
        rate.sleep();
        rate.sleep();
        rate.sleep();
        rate.sleep();

        std::cout << "Connection established" << std::endl;
        std::string v_msg = std::to_string(0.0) + " " + 
                            std::to_string(0.0) + " " + 
                            std::to_string(0.0) + " " + 
                            std::to_string(0.0) + "\r";

        std::cout << "Sending: " << v_msg << std::endl;
        std::cout << "Message length: " << strlen(v_msg.c_str()) << std::endl;

        send(new_socket, v_msg.c_str(), strlen(v_msg.c_str()), 0);
    }

    void SimulinkSocketInterface::on_timer()
    {
        //Wait for TCP message and publish
        std::cout << "Timer: " << std::endl;
        char buffer[1024] = {0};
        read(new_socket, buffer, 127); //127

        std::cout << "Read: " << buffer << std::endl;

        std::vector<float> v;
        std::istringstream iss(buffer);

        std::copy(std::istream_iterator<float>(iss),
                  std::istream_iterator<float>(),
                  std::back_inserter(v));

        auto pos_msg = pr_msgs::msg::PRArrayH();

        for(int i=0; i<4; i++)
            pos_msg.data[i] = v[i];

        pos_msg.current_time = this->get_clock()->now();
        pos_msg.header.stamp = pos_msg.current_time;

        publisher_->publish(pos_msg);
    }

    void SimulinkSocketInterface::topic_callback(const pr_msgs::msg::PRArrayH::SharedPtr control_action_msg)
    {
        std::cout << "Callback " << std::endl;

        double volts[4];

        volts[0] = control_action_msg->data[0]/vp_conversion[0];
        volts[1] = control_action_msg->data[1]/vp_conversion[1];
        volts[2] = control_action_msg->data[2]/vp_conversion[2];
        volts[3] = control_action_msg->data[3]/vp_conversion[3];

        std::string v_msg = std::to_string(volts[0]) + " " + 
                            std::to_string(volts[1]) + " " + 
                            std::to_string(volts[2]) + " " + 
                            std::to_string(volts[3]) + "\r";
        
        std::cout << "Message length: " << strlen(v_msg.c_str()) << std::endl;

        send(new_socket, v_msg.c_str(), strlen(v_msg.c_str()), 0);
    }

    void SimulinkSocketInterface::sat_ca(double &control_action, const double &sat)
    {
	     if(control_action > sat) control_action = sat;
	     if(control_action < -sat) control_action = -sat;
    }

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_sensors_actuators::SimulinkSocketInterface)
