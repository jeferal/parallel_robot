#include "pr_controllers/gus_controller.hpp"

#include <chrono>
#include <memory>
#include <utility>
#include <array>
#include <vector>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "pr_msgs/msg/pr_array_h.hpp"


namespace pr_controllers
{
    /**** GUS CONTROLLER COMPONENT ****/
    GusController::GusController(const rclcpp::NodeOptions & options)
    : Node("gus_controller", options)
    {
        //Parameter declaration
        this->declare_parameter<double>("k1", 0.75);
        this->declare_parameter<double>("k2", 0.5);
        this->declare_parameter<double>("ts", 0.01);

        this->get_parameter("k1", k1);
        this->get_parameter("k2", k2);
        this->get_parameter("ts", ts);

        RCLCPP_INFO(this->get_logger(), "Creating communication");

        sub_ref.subscribe(this, "ref_pose");
        sub_pos.subscribe(this, "joint_position");
        sub_vel.subscribe(this, "joint_velocity");

        sync_.reset(new Synchronizer(SyncPolicy(1), sub_ref, sub_pos, sub_vel));
        sync_->registerCallback(std::bind(&GusController::controller_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        sync_->setMaxIntervalDuration(rclcpp::Duration(0, 10000000));

        publisher_ = this->create_publisher<pr_msgs::msg::PRArrayH>("control_action", 1);

        
    }

    void GusController::controller_callback(const pr_msgs::msg::PRArrayH::ConstPtr& ref_msg,
                                            const pr_msgs::msg::PRArrayH::ConstPtr& pos_msg,
                                            const pr_msgs::msg::PRArrayH::ConstPtr& vel_msg)
    {
        auto control_action_msg = pr_msgs::msg::PRArrayH();
        RCLCPP_INFO(this->get_logger(), "I heard: %f, %f, %f, %f", ref_msg->data[0],
                                                                   pos_msg->data[0],
                                                                   vel_msg->data[0]);

        control_action_msg.header.stamp = this->get_clock()->now();
        control_action_msg.data = ref_msg->data;

        publisher_->publish(control_action_msg);
    }


}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_controllers::GusController)