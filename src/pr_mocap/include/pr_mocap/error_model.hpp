#ifndef PR_MOCAP__ERROR_MODEL_HPP_
#define PR_MOCAP__ERROR_MODEL_HPP_

#include "rclcpp/rclcpp.hpp"
#include "pr_msgs/msg/pr_array_h.hpp"
#include "pr_msgs/msg/pr_mocap.hpp"

#include <array>
#include <vector>

namespace pr_mocap
{
    class ErrorModel : public rclcpp::Node
    {
        public:
            //PR_AUX_PUBLIC
            explicit ErrorModel(const rclcpp::NodeOptions & options);

        protected:
            void model_callback(const pr_msgs::msg::PRArrayH::SharedPtr x_model_msg);
            void mocap_callback(const pr_msgs::msg::PRMocap::SharedPtr x_mocap_msg);
            int error_calc( const double &tol, 
                            double &error, 
                            const std::array<double, 4> &x_mocap, 
                            const std::array<double, 4> &x_model);

        private:
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr subscription_model_;
            rclcpp::Subscription<pr_msgs::msg::PRMocap>::SharedPtr subscription_mocap_;
            rclcpp::Publisher<pr_msgs::msg::PRMocap>::SharedPtr publisher_info_;
            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_mocap_;

            double tol;
            bool is_connected;
            pr_msgs::msg::PRMocap x_mocap;
    };
}

#endif // PR_MOCAP__ERROR_MODEL_HPP_