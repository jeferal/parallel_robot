#include "pr_aux/derivator.hpp"

#include <chrono>
#include <memory>
#include <utility>
#include <array>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "pr_msgs/msg/pr_array_h.hpp"

using std::placeholders::_1;


namespace pr_aux
{
    /**** DERIVATOR COMPONENT ****/
    Derivator::Derivator(const rclcpp::NodeOptions & options)
    : Node("derivator", options)
    {
        //Parameter declaration
        this->declare_parameter<std::vector<double>>("initial_value", {0.679005, 0.708169, 0.684298, 0.637145});
        this->declare_parameter<double>("ts", 0.01);

        this->get_parameter("initial_value", init_val);
        this->get_parameter("ts", ts);

        for(int i=0; i<4; i++)
            var_ant[i] = init_val[i];
        
        publisher_ = this->create_publisher<pr_msgs::msg::PRArrayH>(
			"joint_velocity", 
			1);
        subscription_ = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "joint_position",
            1,
            std::bind(&Derivator::topic_callback, this, _1));
    }

    void Derivator::topic_callback(const pr_msgs::msg::PRArrayH::SharedPtr var_msg)
    {
        auto var_der_msg = pr_msgs::msg::PRArrayH();

        for(int i=0; i<4; i++)
            var_der_msg.data[i] = (var_msg->data[i] - var_ant[i])/ts;

        var_der_msg.header.stamp = this->get_clock()->now();
        var_der_msg.header.frame_id = var_msg->header.frame_id;
        publisher_->publish(var_der_msg);

        var_ant = var_msg->data;
    }
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_aux::Derivator)