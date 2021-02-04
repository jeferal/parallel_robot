#include "pr_sensors_actuators/simulated/motors_simulink.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>
#include <string>

#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;


namespace pr_sensors_actuators
{
    /**** MOTOR SIMULINK COMPONENT ****/
    
    MotorsSimulink::MotorsSimulink(const rclcpp::NodeOptions & options)
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
            std::bind(&MotorsSimulink::topic_callback,this,_1));

        //Voltage publisher
        publisher_ = this->create_publisher<geometry_msgs::msg::Quaternion>(
            "voltaje_sim",
            10
        );

        RCLCPP_INFO(this->get_logger(), "Motors initialized");

    }

    MotorsSimulink::~MotorsSimulink()
    {
        auto volts_msg = geometry_msgs::msg::Quaternion();

        volts_msg.x = 0.0;
        volts_msg.y = 0.0;
        volts_msg.z = 0.0;
        volts_msg.w = 0.0;

        publisher_->publish(volts_msg);
    }

    void MotorsSimulink::topic_callback(const pr_msgs::msg::PRArrayH::SharedPtr control_action_msg) 
    {
        
        volts[0] = control_action_msg->data[0]/vp_conversion[0];
        volts[1] = control_action_msg->data[1]/vp_conversion[1];
        volts[2] = control_action_msg->data[2]/vp_conversion[2];
        volts[3] = control_action_msg->data[3]/vp_conversion[3];

        sat_ca(volts[0], max_v);
        sat_ca(volts[1], max_v);
        sat_ca(volts[2], max_v);
        sat_ca(volts[3], max_v);

        auto volts_msg = geometry_msgs::msg::Quaternion();

        volts_msg.x = volts[0];
        volts_msg.y = volts[1];
        volts_msg.z = volts[2];
        volts_msg.w = volts[3];

        publisher_->publish(volts_msg);
    
    }

    void MotorsSimulink::sat_ca(double &control_action, const double &sat)
    {
	     if(control_action > sat) control_action = sat;
	     if(control_action < -sat) control_action = -sat;
    }

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_sensors_actuators::MotorsSimulink)