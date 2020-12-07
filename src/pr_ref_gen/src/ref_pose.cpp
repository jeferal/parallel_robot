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
            "/home/paralelo4dofnew/aux_ws/referencias/ref_cart_TRR2_v2_CF1.txt");
        
        this->declare_parameter<bool>("is_cart", false);
        this->declare_parameter<std::vector<double>>("robot_config_params", 
            {0.4, 0.4, 0.4, 0.15, 90*(M_PI/180), 45*(M_PI/180), 0.3, 0.3, 0.3, 50*(M_PI/180), 90*(M_PI/180)});

        this->get_parameter("ref_path", ref_path);
        this->get_parameter("is_cart", is_cart);
        this->get_parameter("robot_config_params", robot_params);

        //Read file
        if(!PRUtils::read_file(ref_matrix, ref_path)){
            RCLCPP_ERROR(this->get_logger(), "Could not open file");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Pose references file opened");
        n_ref = ref_matrix.rows();
    }

}