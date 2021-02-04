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
    /**** ENCODERS SIMULINK COMPONENT ****/
    
    MotorsSimulink::MotorsSimulink(const rclcpp::NodeOptions & options)
    : Node("position_sensors", options)
    {
        //Parameter declaration
        this->declare_parameter<double>("max_v", 3.0);
        this->declare_parameter<double>("vp_conversion", 1.0);

        //Read parameters
        this->get_parameter("vp_conversion", vp_conversion);
        this->get_parameter("max_v", max_v);

        //Voltage publisher

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
        volts[0] = control_action_msg->data[0]/vp_conversion;
        volts[1] = control_action_msg->data[1]/vp_conversion;
        volts[2] = control_action_msg->data[2]/vp_conversion;
        volts[3] = control_action_msg->data[3]/vp_conversion;

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