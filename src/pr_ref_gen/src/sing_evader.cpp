#include "pr_ref_gen/sing_evader.hpp"

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


namespace pr_ref_gen
{
    /**** Singularity CONTROLLER COMPONENT ****/
    SingEvader::SingEvader(const rclcpp::NodeOptions & options)
    : Node("sin_evader", options)
    {
        //Parameter declaration

        RCLCPP_INFO(this->get_logger(), "Creating communication");

        sub_ref.subscribe(this, "ref_pose");
        sub_x.subscribe(this, "x_coord");
        sub_ots.subscribe(this, "ang_ots");

        sync_.reset(new Synchronizer(SyncPolicy(1), sub_ref, sub_x, sub_ots));
        sync_->registerCallback(std::bind(&SingEvader::topic_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        publisher_ = this->create_publisher<pr_msgs::msg::PRArrayH>("ref_pose_mod", 1);


    }

    void SingEvader::topic_callback(const pr_msgs::msg::PRArrayH::ConstPtr& ref_msg,
                                    const pr_msgs::msg::PRArrayH::ConstPtr& x_msg,
                                    const pr_msgs::msg::PROTS::ConstPtr& ots_msg)
    {
        RCLCPP_INFO(this->get_logger(), "Singularity evader callback");
    }


}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_ref_gen::SingEvader)