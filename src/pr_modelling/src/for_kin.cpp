#include "pr_modelling/for_kin.hpp"

#include <chrono>
#include <memory>
#include <utility>
#include <array>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "pr_msgs/msg/pr_array_h.hpp"
#include "pr_lib/pr_model.hpp"
#include "pr_lib/pr_utils.hpp"

using std::placeholders::_1;

namespace pr_modelling
{
    /**** FORWARD KINEMATICS COMPONENT ****/
    ForwardKinematics::ForwardKinematics(const rclcpp::NodeOptions & options)
    : Node("for_kin", options)
    {
        //Parameter declaration
        this->declare_parameter<std::vector<double>>(
            "robot_config_params", 
            {0.4, 0.4, 0.4, 0.15, 90*(M_PI/180), 45*(M_PI/180), 0.3, 0.3, 0.3, 50*(M_PI/180), 90*(M_PI/180)});
        
        this->declare_parameter<std::vector<double>>(
            "initial_position", 
            {0.069190, 0.632000, 0.078714, -0.135787});

        this->declare_parameter<double>("tol", 0.0000007);
        this->declare_parameter<int>("iter", 30);

        this->get_parameter("robot_config_params", robot_params);
        this->get_parameter("initial_position", x_prev);
        this->get_parameter("tol", tol);
        this->get_parameter("iter", iter);

        publisher_ = this->create_publisher<pr_msgs::msg::PRArrayH>(
            "x_coord", 
            10);

        subscription_ = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "joint_position", 
            10, 
            std::bind(&ForwardKinematics::topic_callback, this, _1));
    }

    void ForwardKinematics::topic_callback(const pr_msgs::msg::PRArrayH::SharedPtr q_msg)
    {
        auto x_sol_msg = pr_msgs::msg::PRArrayH();

        x_sol_msg.data = PRModel::ForwardKinematics(q_msg->data, x_prev, robot_params, tol, iter);

        PRUtils::array2vector(x_sol_msg.data, x_prev);

        x_sol_msg.header.stamp = this->get_clock()->now();
            
        publisher_->publish(x_sol_msg);
    }

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_modelling::ForwardKinematics)