#include "pr_modelling/state_publisher.hpp"

#include <chrono>
#include <memory>
#include <utility>
#include <array>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

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

        publisher_ = this->create_publisher<pr_msgs::msg::PRState>("pr_state", 1);
    }

    void StatePublisher::topic_callback(const pr_msgs::msg::PRArrayH::ConstPtr& q_msg,
                                           const pr_msgs::msg::PRArrayH::ConstPtr& q_vel_msg)
    {
        auto state_msg = pr_msgs::msg::PRState();

        state_msg.q.data = q_msg->data;
        state_msg.q_vel.data = q_vel_msg->data;

        state_msg.current_time = this->get_clock()->now();
        state_msg.header.stamp = q_msg->header.stamp;
        state_msg.header.frame_id = q_msg->header.frame_id + ", " + q_vel_msg->header.frame_id;

        publisher_->publish(state_msg);
    }
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_modelling::StatePublisher)