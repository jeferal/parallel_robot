#include "pr_modelling/state_publisher.hpp"

#include <chrono>
#include <memory>
#include <utility>
#include <array>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "pr_msgs/msg/pr_array_h.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace pr_modelling
{
    /**** DEPENDENT JACOBIAN COMPONENT ****/
    StatePublisher::StatePublisher(const rclcpp::NodeOptions & options)
    : Node("state_publisher", options)
    {

        sub_q.subscribe(this, "joint_position");
        sub_q_vel.subscribe(this, "joint_velocity");
        sync_.reset(new Synchronizer(SyncPolicy(1), sub_q, sub_q_vel));
        sync_->registerCallback(std::bind(&StatePublisher::topic_callback, this, _1, _2));
        //sync_->setMaxIntervalDuration(rclcpp::Duration(0, 1000000));

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

        jac_dep_msg.current_time = this->get_clock()->now();
        jac_dep_msg.header.stamp = x_coord_msg->header.stamp;
        jac_dep_msg.header.frame_id = x_coord_msg->header.frame_id + ", " + q_msg->header.frame_id;

        publisher_->publish(jac_dep_msg);
    }
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_modelling::DependentJacobian)