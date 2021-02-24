#include "pr_modelling/ang_ots.hpp"

#include <chrono>
#include <memory>
#include <utility>
#include <array>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "pr_msgs/msg/pr_mat_h.hpp"

using std::placeholders::_1;

namespace pr_modelling
{
    /**** ANG OTS COMPONENT ****/
    AngOTS::AngOTS(const rclcpp::NodeOptions & options)
    : Node("ang_ots", options)
    {
        //Declare params
        this->declare_parameter<std::vector<double>>(
            "robot_config_params", 
            {0.4, 0.4, 0.4, 0.15, 90*(M_PI/180), 45*(M_PI/180), 0.3, 0.3, 0.3, 50*(M_PI/180), 90*(M_PI/180)});
        
        
        this->get_parameter("robot_config_params", robot_params);

        publisher_ = this->create_publisher<pr_msgs::msg::PRArrayH>(
            "ang_ots", 
            10);

        subscription_ = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "x_coord_cams", 
            10, 
            std::bind(&AngOTS::topic_callback, this, _1));
    }

    void AngOTS::topic_callback(const pr_msgs::msg::PRArrayH::SharedPtr x_msg)
    {
        //Calculate inverse kinematics
        PRModel::InverseKinematics(q_sol, x_msg->data, robot_params);

        //
        
    }
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_modelling::AngOTS)