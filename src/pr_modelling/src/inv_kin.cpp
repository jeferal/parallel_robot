#include "pr_modelling/inv_kin.hpp"

#include <chrono>
#include <memory>
#include <utility>
#include <array>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "pr_msgs/msg/pr_array_h.hpp"
#include "pr_msgs/msg/pr_mat_h.hpp"

#include "pr_lib/pr_model.hpp"
#include "pr_lib/pr_utils.hpp"

using std::placeholders::_1;

namespace pr_modelling
{
    /**** INVERSE KINEMATICS COMPONENT ****/
    InverseKinematics::InverseKinematics(const rclcpp::NodeOptions & options)
    : Node("inv_kin", options)
    {
        //Parameter declaration
        this->declare_parameter<std::vector<double>>(
            "robot_config_params", 
            {0.4, 0.4, 0.4, 0.15, 90*(M_PI/180), 45*(M_PI/180), 0.3, 0.3, 0.3, 50*(M_PI/180), 90*(M_PI/180)});
        
        this->get_parameter("robot_config_params", robot_params);

        publisher_ = this->create_publisher<pr_msgs::msg::PRMatH>(
            "q_sol", 
            10);

        subscription_ = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "x_coord", 
            10, 
            std::bind(&InverseKinematics::topic_callback, this, _1));
    }

    void InverseKinematics::topic_callback(const pr_msgs::msg::PRArrayH::SharedPtr x_msg)
    {
        auto q_sol_msg = pr_msgs::msg::PRMatH();
        
        PRModel::InverseKinematics(q_sol, x_msg->data, robot_params);

        PRUtils::Eigen2MatMsg(q_sol, q_sol_msg);

        //q_sol_msg.header.stamp = this->get_clock()->now();
        q_sol_msg.header.stamp = x_msg->header.stamp;
        q_sol_msg.header.frame_id = x_msg->header.frame_id;
        
        publisher_->publish(q_sol_msg);
    }
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_modelling::InverseKinematics)