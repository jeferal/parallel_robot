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

using std::placeholders::_1;

namespace pr_modelling
{
    /**** INDEPENDENT JACOBIAN COMPONENT ****/
    IndependentJacobian::IndependentJacobian(const rclcpp::NodeOptions & options)
    : Node("ind_jac", options)
    {

        publisher_ = this->create_publisher<pr_msgs::msg::PRMatH>(
            "q_sol", 
            10);

        subscription_ = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "x_coor", 
            10, 
            std::bind(&IndependentJacobian::topic_callback, this, _1));
    }

    void IndependentJacobian::topic_callback(const pr_msgs::msg::PRArrayH::SharedPtr x_msg)
    {
        auto ind_j_msg = pr_msgs::msg::PRMatH();

        //Conversión data to eigen (hacer con función e incluir en el mensaje rows y cols)
        for(int i=0; i<4; i++){
            for(int j=0; j<3; j++)
                Q(i,j) = x_msg->data[i*j];
        }

        PRModel::IndJacobian(IndJ, Q);        

        //Conversión eigen to data (hacer con función e incluir en el mensaje rows y cols)
        for(int i=0; i<11; i++){
            for(int j=0; j<4; j++)
                IndJ(i,j) = ind_j_msg.data[i*j];
        }

        ind_j_msg.header.stamp = this->get_clock()->now();
        publisher_->publish(ind_j_msg);
    }
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_modelling::IndependentJacobian)