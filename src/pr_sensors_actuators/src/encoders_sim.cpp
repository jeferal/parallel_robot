#include "pr_sensors_actuators/encoders_sim.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "pr_msgs/msg/pr_array_h.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace pr_sensors_actuators
{
    /**** ENCODERS COMPONENT ****/
    
    EncodersSim::EncodersSim(const rclcpp::NodeOptions & options)
    : Node("position_sensors", options)
    {
        //Parameter declaration
        this->declare_parameter<float>("ts_ms", 10.0);
        this->declare_parameter<std::vector<double>>("initial_position", {0.665559, 0.654974, 0.691026, 0.631511});


        //Position publisher
        publisher_ = this->create_publisher<pr_msgs::msg::PRArrayH>(
			"joint_position", 
			1);
        
        timer_ = this->create_wall_timer(
            10ms, 
            std::bind(&EncodersSim::on_timer, this));

		RCLCPP_INFO(this->get_logger(), "Configuration completed, brake disabled");
    }

    void EncodersSim::on_timer()
    {
        
		auto position_msg = pr_msgs::msg::PRArrayH();

		position_msg.data[0] = 3.0;
        position_msg.data[1] = 2.0;
        position_msg.data[2] = 1.0;
        position_msg.data[3] = 4.0;

        publisher_->publish(position_msg);
	}


}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_sensors_actuators::EncodersSim)