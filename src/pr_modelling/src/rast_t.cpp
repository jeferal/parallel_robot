#include "pr_modelling/rast_t.hpp"

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
    /**** RastT COMPONENT ****/
    RastT::RastT(const rclcpp::NodeOptions & options)
    : Node("rast_t", options)
    {        
        sub_dep.subscribe(this, "dep_jac");
        sub_ind.subscribe(this, "ind_jac");
        sync_.reset(new Synchronizer(SyncPolicy(1), sub_dep, sub_ind));
        sync_->registerCallback(std::bind(&RastT::topic_callback, this, std::placeholders::_1, std::placeholders::_2));
        sync_->setMaxIntervalDuration(rclcpp::Duration(0, 10000000));

        publisher_ = this->create_publisher<pr_msgs::msg::PRMatH>("rast_t", 1);
    }

    void RastT::topic_callback(const pr_msgs::msg::PRMatH::ConstPtr& jac_dep_msg,
                               const pr_msgs::msg::PRMatH::ConstPtr& jac_ind_msg)
    {
        auto rast_t_msg = pr_msgs::msg::PRMatH();

        //Conversion
        PRUtils::MatMsgR2Eigen(jac_dep_msg, DepJ);
        PRUtils::MatMsgR2Eigen(jac_ind_msg, IndJ);

        PRModel::Rast(Rast, DepJ, IndJ);

        //Conversion y transponer
        PRUtils::Eigen2MatMsgT(Rast, rast_t_msg);

        rast_t_msg.header.stamp = this->get_clock()->now();

        publisher_->publish(rast_t_msg);
    }
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_modelling::RastT)