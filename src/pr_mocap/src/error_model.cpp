#include "pr_mocap/error_model.hpp"

#include <chrono>
#include <memory>
#include <utility>
#include <array>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

namespace pr_mocap
{
    /**** DERIVATOR COMPONENT ****/
    ErrorModel::ErrorModel(const rclcpp::NodeOptions & options)
    : Node("error_model", options)
    {
        //Parameter declaration
        this->declare_parameter<double>("tol", 0.01);
        this->get_parameter("tol", tol);
        
        publisher_ = this->create_publisher<pr_msgs::msg::PRMocap>(
			"x_mocap_error", 
			1);
        
        subscription_mocap_ = this->create_subscription<pr_msgs::msg::PRMocap>(
            "x_coord_mocap",
            1,
            std::bind(&ErrorModel::mocap_callback, this, _1));
        
        subscription_model_ = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "x_coord",
            1,
            std::bind(&ErrorModel::model_callback, this, _1));
    }

    void ErrorModel::mocap_callback(const pr_msgs::msg::PRMocap::SharedPtr x_mocap_msg)
    {
        x_mocap.x_coord.data = x_mocap_msg->x_coord.data;
        x_mocap.latency = x_mocap_msg->latency;
    }

    void ErrorModel::model_callback(const pr_msgs::msg::PRArrayH::SharedPtr x_model_msg)
    {
        auto error_msg = pr_msgs::msg::PRMocap();

        error_calc(tol, error_msg.error, x_mocap.x_coord.data, x_model_msg->data);

        error_msg.header = x_model_msg->header;
        error_msg.current_time = this->get_clock()->now();
        error_msg.latency = x_mocap.latency;
        error_msg.x_coord.data = x_mocap.x_coord.data;

        publisher_->publish(error_msg);
    }

    int ErrorModel::error_calc(const double &tol, double &error, const std::array<double, 4> &x_mocap, const std::array<double, 4> &x_model)
    {
        error = 0.0;
        for(int i=0; i<4; i++)
            error = error + pow(x_mocap[i] - x_model[i], 2);

        return error > tol;
    }
    
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_mocap::ErrorModel)