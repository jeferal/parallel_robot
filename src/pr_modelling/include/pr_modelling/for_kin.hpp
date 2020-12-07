#ifndef PR_MODELLING__FOR_KIN_HPP_
#define PR_MODELLING__FOR_KIN_HPP_

#include "rclcpp/rclcpp.hpp"

#include "pr_msgs/msg/pr_array_h.hpp"
#include "pr_lib/pr_model.hpp"

namespace pr_modelling
{
    class ForwardKinematics : public rclcpp::Node
    {
        public:
            //PR_AUX_PUBLIC
            explicit ForwardKinematics(const rclcpp::NodeOptions & options);

        protected:
            void topic_callback(const pr_msgs::msg::PRArrayH::SharedPtr q_msg);

        private:
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr subscription_;
            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_;
            std::vector<double> robot_params;
            std::vector<double> x_prev;
            double tol;
            int iter;
    };
}

#endif // PR_MODELLING__FOR_KIN_HPP_