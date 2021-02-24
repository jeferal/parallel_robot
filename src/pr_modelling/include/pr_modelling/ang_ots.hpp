#ifndef PR_MODELLING__ANG_OTS_HPP_
#define PR_MODELLING__ANG_OTS_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "pr_msgs/msg/pr_array_h.hpp"
#include "pr_msgs/msg/pr_mat_h.hpp"
#include "pr_lib/pr_singularity.hpp"
#include "pr_lib/pr_model.hpp"

#include "eigen3/Eigen/Dense"

namespace pr_modelling
{
    class AngOTS : public rclcpp::Node
    {
        public:
            //PR_MODELLING_PUBLIC
            explicit AngOTS(const rclcpp::NodeOptions & options);

        protected:
            void topic_callback(const pr_msgs::msg::PRArrayH::SharedPtr x_msg);

        private:
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr subscription_;
            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_;

            std::vector<double> robot_params;
            Eigen::Matrix<double, 4, 3> q_sol = Eigen::Matrix<double, 4, 3>::Zero();

    };
}

#endif // PR_MODELLING__ANG_OTS_HPP_