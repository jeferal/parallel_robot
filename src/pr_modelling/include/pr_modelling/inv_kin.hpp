#ifndef PR_MODELLING__INV_KIN_HPP_
#define PR_MODELLING__INV_KIN_HPP_

#include "rclcpp/rclcpp.hpp"

#include "pr_msgs/msg/pr_array_h.hpp"
#include "pr_msgs/msg/pr_mat_h.hpp"
#include "pr_lib/pr_model.hpp"

namespace pr_modelling
{
    class InverseKinematics : public rclcpp::Node
    {
        public:
            //PR_MODELLING_PUBLIC
            explicit InverseKinematics(const rclcpp::NodeOptions & options);

        protected:
            void topic_callback(const pr_msgs::msg::PRArrayH::SharedPtr x_msg);

        private:
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr subscription_;
            rclcpp::Publisher<pr_msgs::msg::PRMatH>::SharedPtr publisher_;
            std::vector<double> robot_params;
            Eigen::Matrix<double, 4, 3> q_sol = Eigen::Matrix<double, 4, 3>::Zero();
    };
}

#endif // PR_MODELLING__INV_KIN_HPP_