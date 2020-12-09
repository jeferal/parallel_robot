#include "pr_modelling/ind_jac.hpp"

#include <chrono>
#include <memory>
#include <utility>
#include <array>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "pr_msgs/msg/pr_mat_h.hpp"

#include "pr_lib/pr_model.hpp"
#include "pr_lib/pr_utils.hpp"

using std::placeholders::_1;

namespace pr_modelling
{
    /**** INDEPENDENT JACOBIAN COMPONENT ****/
    IndependentJacobian::IndependentJacobian(const rclcpp::NodeOptions & options)
    : Node("ind_jac", options)
    {

        publisher_ = this->create_publisher<pr_msgs::msg::PRMatH>(
            "ind_jac", 
            10);

        subscription_ = this->create_subscription<pr_msgs::msg::PRMatH>(
            "q_sol", 
            10, 
            std::bind(&IndependentJacobian::topic_callback, this, _1));
    }

    void IndependentJacobian::topic_callback(const pr_msgs::msg::PRMatH::SharedPtr q_msg)
    {
        auto ind_j_msg = pr_msgs::msg::PRMatH();

        RCLCPP_INFO(this->get_logger(), "Receiving: %f %f %f %f", 
            q_msg->data[3], 
            q_msg->data[4], 
            q_msg->data[5], 
            q_msg->data[6]);
        
        PRUtils::Mat2Eigen__11_4(q_msg, IndJ);

        PRModel::IndJacobian(IndJ, Q);        

        PRUtils::Eigen2Mat(IndJ, ind_j_msg);

        ind_j_msg.header.stamp = this->get_clock()->now();
        publisher_->publish(ind_j_msg);
    }
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_modelling::IndependentJacobian)