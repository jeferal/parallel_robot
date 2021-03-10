#ifndef PR_MODELLING__FOR_JAC_HPP_
#define PR_MODELLING__FOR_JAC_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "pr_msgs/msg/pr_array_h.hpp"
#include "pr_msgs/msg/pr_float_h.hpp"

#include "pr_lib/pr_model.hpp"

namespace pr_modelling
{
    class ForwardJacobian : public rclcpp::Node
    {
        public:
            //PR_MODELLING_PUBLIC
            explicit ForwardJacobian(const rclcpp::NodeOptions & options);

        protected:
            void topic_callback(const pr_msgs::msg::PRArrayH::SharedPtr x_msg);

        private:
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr subscription_;
            rclcpp::Publisher<pr_msgs::msg::PRFloatH>::SharedPtr publisher_;

            Eigen::Matrix<double, 4, 4> ForJac = Eigen::Matrix<double, 4, 4>::Zero();
            std::vector<double> robot_params;
    };
}

#endif // PR_MODELLING__IND_JAC_HPP_