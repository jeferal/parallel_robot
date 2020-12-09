#ifndef PR_MODELLING__IND_JAC_HPP_
#define PR_MODELLING__IND_JAC_HPP_

#include "rclcpp/rclcpp.hpp"

#include "pr_msgs/msg/pr_array_h.hpp"
#include "pr_msgs/msg/pr_mat_h.hpp"
#include "pr_lib/pr_model.hpp"

namespace pr_modelling
{
    class IndependentJacobian : public rclcpp::Node
    {
        public:
            //PR_AUX_PUBLIC
            explicit IndependentJacobian(const rclcpp::NodeOptions & options);

        protected:
            void topic_callback(const pr_msgs::msg::PRMatH::SharedPtr x_msg);

        private:
            rclcpp::Subscription<pr_msgs::msg::PRMatH>::SharedPtr subscription_;
            rclcpp::Publisher<pr_msgs::msg::PRMatH>::SharedPtr publisher_;
            Eigen::Matrix<double, 11, 4> IndJ = Eigen::Matrix<double, 11, 4>::Zero();
            Eigen::Matrix<double, 4, 3> Q = Eigen::Matrix<double, 4, 3>::Zero();
    };
}

#endif // PR_MODELLING__IND_JAC_HPP_