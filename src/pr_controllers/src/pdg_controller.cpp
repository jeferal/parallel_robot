#include "pr_controllers/pdg_controller.hpp"

#include <chrono>
#include <memory>
#include <utility>
#include <array>
#include <vector>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "pr_msgs/msg/pr_array_h.hpp"
#include "pr_lib/pr_utils.hpp"


namespace pr_controllers
{
    /**** PDG CONTROLLER COMPONENT ****/
    PDGController::PDGController(const rclcpp::NodeOptions & options)
    : Node("pdg_controller", options)
    {
        //Parameter declaration
        this->declare_parameter<std::vector<double>>("kp_gain", {27190.0, 27190.0, 27190.0, 361023.0});
        this->declare_parameter<std::vector<double>>("kv_gain", {114.27, 114.27, 114.27, 491.32});

        this->get_parameter("kp_gain", Kp);
        this->get_parameter("kv_gain", Kv);

        RCLCPP_INFO(this->get_logger(), "Creating communication");

        sub_ref.subscribe(this, "ref_pose");
        sub_pos.subscribe(this, "joint_position");
        sub_vel.subscribe(this, "joint_velocity");
        sub_grav.subscribe(this, "q_grav");

        sync_.reset(new Synchronizer(SyncPolicy(1), sub_ref, sub_pos, sub_vel, sub_grav));
        sync_->registerCallback(std::bind(&PDGController::controller_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
        //sync_->setMaxIntervalDuration(rclcpp::Duration(0, 8000000));

        publisher_ = this->create_publisher<pr_msgs::msg::PRArrayH>("control_action", 1);


    }

    void PDGController::controller_callback(const pr_msgs::msg::PRArrayH::ConstPtr& ref_msg,
                                            const pr_msgs::msg::PRArrayH::ConstPtr& pos_msg,
                                            const pr_msgs::msg::PRArrayH::ConstPtr& vel_msg,
                                            const pr_msgs::msg::PRArrayH::ConstPtr& grav_msg)
    {
        auto control_action_msg = pr_msgs::msg::PRArrayH();


        //Calculate control action
        for(int i=0; i<4; i++)
            control_action_msg.data[i] = Kp[i]*(ref_msg->data[i] - pos_msg->data[i]) - Kv[i]*vel_msg->data[i] + grav_msg->data[i];

        control_action_msg.header.stamp = this->get_clock()->now();
        control_action_msg.header.frame_id = pos_msg->header.frame_id + ", " + ref_msg->header.frame_id + ", " + vel_msg->header.frame_id + ", " + grav_msg->header.frame_id;

        publisher_->publish(control_action_msg);

        /*RCLCPP_INFO(this->get_logger(), "I heard: %f, %f, %f, %f", control_action_msg.data[0],
                                                                   control_action_msg.data[1],
                                                                   control_action_msg.data[2],
                                                                   control_action_msg.data[3]);
        */
    }


}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_controllers::PDGController)