#ifndef PR_AUX__REPLAYER_HPP_
#define PR_AUX__REPLAYER_HPP_

#include "rclcpp/rclcpp.hpp"

#include "pr_msgs/msg/pr_array_h.hpp"
#include "std_msgs/msg/bool.hpp"

#include "eigen3/Eigen/Dense"

namespace pr_aux
{
    class Replayer : public rclcpp::Node
    {
        public:
            //PR_REF_GEN_PUBLIC
            explicit Replayer(const rclcpp::NodeOptions & options);

        protected:
            void on_timer();
            void end_callback(const std_msgs::msg::Bool::SharedPtr end_msg);

        private:
            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_;
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_end_;
            rclcpp::TimerBase::SharedPtr timer_;

            std::string data_path;
            Eigen::MatrixXd data_matrix;
            bool is_finished = false;
            double ts_ms;
            int n_data;
            int idx=0;
    };
}

#endif // PR_AUX__REPLAYER_HPP_