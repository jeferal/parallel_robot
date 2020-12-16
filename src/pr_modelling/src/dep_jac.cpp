#include "pr_modelling/dep_jac.hpp"

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
    /**** DEPENDENT JACOBIAN COMPONENT ****/
    DependentJacobian::DependentJacobian(const rclcpp::NodeOptions & options)
    : Node("dep_jac", options)
    {
        //Parameter declaration
        this->declare_parameter<std::vector<double>>(
            "robot_config_params", 
            {0.4, 0.4, 0.4, 0.15, 90*(M_PI/180), 45*(M_PI/180), 0.3, 0.3, 0.3, 50*(M_PI/180), 90*(M_PI/180)});
        
        this->get_parameter("robot_config_params", robot_params);

        sub_x.subscribe(this, "x_coord");
        sub_q.subscribe(this, "q_sol");
        sync_.reset(new Synchronizer(SyncPolicy(1), sub_x, sub_q));
        sync_->registerCallback(std::bind(&DependentJacobian::topic_callback, this, std::placeholders::_1, std::placeholders::_2));
        sync_->setMaxIntervalDuration(rclcpp::Duration(0, 1000000));

        publisher_ = this->create_publisher<pr_msgs::msg::PRMatH>("dep_jac", 1);
    }

    void DependentJacobian::topic_callback(const pr_msgs::msg::PRArrayH::ConstPtr& x_coord_msg,
                                           const pr_msgs::msg::PRMatH::ConstPtr& q_msg)
    {
        auto jac_dep_msg = pr_msgs::msg::PRMatH();

        PRUtils::MatMsgR2Eigen(q_msg, Q);

        PRUtils::MatMsgR2Eigen(q_msg, Q);     
        PRModel::DepJacobian(DepJ, Q, x_coord_msg->data[2], x_coord_msg->data[3], robot_params);

        PRUtils::Eigen2MatMsg(DepJ, jac_dep_msg);

        jac_dep_msg.header.stamp = this->get_clock()->now();
        jac_dep_msg.header.frame_id = x_coord_msg->header.frame_id + ", " + q_msg->header.frame_id;

        publisher_->publish(jac_dep_msg);
    }
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_modelling::DependentJacobian)