#include "pr_modelling/q_grav.hpp"

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
    /**** Q GRAV TERMS COMPONENT ****/
    QGrav::QGrav(const rclcpp::NodeOptions & options)
    : Node("q_grav", options)
    {
        //Parameter declaration
        this->declare_parameter<std::vector<double>>(
            "robot_config_params", 
            {0.4, 0.4, 0.4, 0.15, 90*(M_PI/180), 45*(M_PI/180), 0.3, 0.3, 0.3, 50*(M_PI/180), 90*(M_PI/180)});
        
        this->get_parameter("robot_config_params", robot_params);

        sub_x.subscribe(this, "x_coord");
        sub_q.subscribe(this, "q_sol");
        sub_rast_t.subscribe(this, "rast_t");

        sync_.reset(new Synchronizer(SyncPolicy(1), sub_x, sub_q, sub_rast_t));
        sync_->registerCallback(std::bind(&QGrav::topic_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        sync_->setMaxIntervalDuration(rclcpp::Duration(0, 10000000));

        publisher_ = this->create_publisher<pr_msgs::msg::PRArrayH>("q_grav", 1);
    }

    void QGrav::topic_callback(const pr_msgs::msg::PRArrayH::ConstPtr& x_msg,
                               const pr_msgs::msg::PRMatH::ConstPtr& q_msg,
                               const pr_msgs::msg::PRMatH::ConstPtr& rast_t_msg)
    {

        auto q_grav_msg = pr_msgs::msg::PRArrayH();

        PRUtils::MatMsgR2Eigen(q_msg, Q);
        PRUtils::MatMsgR2Eigen(rast_t_msg, RastT);

        PRModel::QGravFunction(QGravTerms, RastT, x_msg->data[2], x_msg->data[3], Q, 
                               P11, P12, P21, P22, P31, P32, P41, P42, Pm);

        PRUtils::Eigen2ArMsg(QGravTerms, q_grav_msg);

        q_grav_msg.header.stamp = this->get_clock()->now();

        publisher_->publish(q_grav_msg);
    }
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_modelling::QGrav)