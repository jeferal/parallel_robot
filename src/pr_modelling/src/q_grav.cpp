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
        this->declare_parameter<std::vector<double>>("p11", {8.57287, -0.03389, 0.00102, 0.19460,  0.17087, -0.00050, 0.01739, 0.18719, 0.00007, 0.02096});
        this->declare_parameter<std::vector<double>>("p12", {0.86261, 0.00000, 0.00000, -0.26250, 0.02278, 0.00000, 0.00000, 0.02278, 0.00000, 0.00004});
        this->declare_parameter<std::vector<double>>("p21", {8.57287, 0.03389, -0.00102, 0.19460, 0.17087, -0.00050, -0.01739, 0.18719, -0.00007, 0.02096});
        this->declare_parameter<std::vector<double>>("p22", {0.86261, 0.00000, 0.00000, -0.26250, 0.02278, 0.00000, 0.00000, 0.02278, 0.00000, 0.00004});
        this->declare_parameter<std::vector<double>>("p31", {8.57287, 0.03389, -0.00102, 0.19460, 0.17087, -0.00050, -0.01739, 0.18719, -0.00007, 0.02096});
        this->declare_parameter<std::vector<double>>("p32", {0.86261, 0.00000, 0.00000, -0.26250, 0.02278, 0.00000, 0.00000, 0.02278, 0.00000, 0.00004});
        this->declare_parameter<std::vector<double>>("p41", {8.81275, 0.00098, 0.01742, 0.13758, 0.09592, 0.00044, 0.00021, 0.08853, -0.00208, 0.01168});
        this->declare_parameter<std::vector<double>>("p42", {1.73524, 0.00000, 0.15148, 0.00000, 0.03115, 0.00000, 0.00000, 0.00056, 0.00000, 0.03091});
        this->declare_parameter<std::vector<double>>("pm", {9.82033, 0.00393, -0.00181, 0.03710, 0.26398, 0.01131, 0.00083, 0.21707, -0.00038, 0.47887});

        this->get_parameter("p11", P11);
        this->get_parameter("p12", P12);
        this->get_parameter("p21", P21);
        this->get_parameter("p22", P22);
        this->get_parameter("p31", P31);
        this->get_parameter("p32", P32);
        this->get_parameter("p41", P41);
        this->get_parameter("p42", P42);
        this->get_parameter("pm", Pm);

        sub_x.subscribe(this, "x_coord");
        sub_q.subscribe(this, "q_sol");
        sub_rast_t.subscribe(this, "rast_t");

        sync_.reset(new Synchronizer(SyncPolicy(1), sub_x, sub_q, sub_rast_t));
        sync_->registerCallback(std::bind(&QGrav::topic_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        //sync_->setMaxIntervalDuration(rclcpp::Duration(0, 1000000));

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

        q_grav_msg.current_time = this->get_clock()->now();
        q_grav_msg.header.stamp = x_msg->header.stamp;
        q_grav_msg.header.frame_id = x_msg->header.frame_id + ", " + q_msg->header.frame_id + ", " + rast_t_msg->header.frame_id; 

        publisher_->publish(q_grav_msg);
    }
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_modelling::QGrav)