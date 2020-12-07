#include "pr_ref_gen/ref_pose.hpp"

#include <chrono>
#include <memory>
#include <utility>
#include <array>
#include <vector>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "pr_msgs/msg/pr_array_h.hpp"

#include "pr_lib/pr_model.hpp"
#include "pr_lib/pr_utils.hpp"

using std::placeholders::_1;

namespace pr_ref_gen
{
    /**** REFERENCE POSE GENERATOR COMPONENT ****/
    RefPose::RefPose(const rclcpp::NodeOptions & options)
    : Node("ref_pose", options)
    {
        //Parameter declaration
        this->declare_parameter<std::string>("ref_path", 
            "/home/paralelo4dofnew/parallel_robot_ws/references/ref_cart_TRR0_CF1_IdV1.txt");
        
        this->declare_parameter<bool>("is_cart", true);
        this->declare_parameter<std::vector<double>>("robot_config_params", 
            {0.4, 0.4, 0.4, 0.15, 90*(M_PI/180), 45*(M_PI/180), 0.3, 0.3, 0.3, 50*(M_PI/180), 90*(M_PI/180)});

        this->get_parameter("ref_path", ref_path);
        this->get_parameter("is_cart", is_cart);
        this->get_parameter("robot_config_params", robot_params);

        //Read file
        if(PRUtils::read_file(ref_matrix, ref_path)==-1){
            RCLCPP_ERROR(this->get_logger(), "Could not open file");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Pose references file opened");
        n_ref = ref_matrix.rows();

        if(is_cart)
        {
            //Cartesian to joint conversion (Inverse kinematics)
            for(int i=0; i<n_ref; i++)
            {
                Eigen::RowVector4d q_row;
                Eigen::RowVector4d x_row = ref_matrix.row(i);
                PRModel::InverseKinematicsPrism(q_row, x_row, robot_params);
                ref_matrix.row(i) = q_row; 
            }
        }

        //Create communication
        publisher_ = this->create_publisher<pr_msgs::msg::PRArrayH>(
            "ref_pose",
            1
        );

        publisher_end_ = this->create_publisher<std_msgs::msg::Bool>(
            "end_flag",
            1
        );

        subscription_ = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "joint_position",
            1,
            std::bind(&RefPose::topic_callback, this, _1)
        );

        
    }

    void RefPose::topic_callback(const pr_msgs::msg::PRArrayH::SharedPtr q_msg)
    {
        if(idx<n_ref)
        {
            auto ref_msg = pr_msgs::msg::PRArrayH();
            //CONVERTIR A FUNCIÓN
            for(int i=0; i<4; i++)
                ref_msg.data[i] = ref_matrix(idx, i);

            ref_msg.header.stamp = this->get_clock()->now();
            publisher_->publish(ref_msg); 
        }
        else
        {
            auto end_msg = std_msgs::msg::Bool();
            end_msg.data = true;
            publisher_end_->publish(end_msg);
        }

        idx++;
    }

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_ref_gen::RefPose)