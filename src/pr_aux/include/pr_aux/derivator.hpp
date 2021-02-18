#ifndef PR_AUX__DERIVATOR_HPP_
#define PR_AUX__DERIVATOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "pr_msgs/msg/pr_array_h.hpp"
#include "std_msgs/msg/bool.hpp"

#include <array>
#include <vector>

namespace pr_aux
{
    class Derivator : public rclcpp::Node
    {
        public:
            //PR_AUX_PUBLIC
            explicit Derivator(const rclcpp::NodeOptions & options);

        protected:
            void topic_callback(const pr_msgs::msg::PRArrayH::SharedPtr var_msg);
            void reset_callback(const std_msgs::msg::Bool::SharedPtr reset_msg);

        private:
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr subscription_;
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_reset_;
            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_;
            double ts;
            std::array<double, 4> var_ant;
            std::vector<double> init_val;
    };
}

#endif // PR_AUX__MOTOR_HPP_