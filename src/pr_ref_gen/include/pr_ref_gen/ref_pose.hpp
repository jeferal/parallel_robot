#ifndef PR_REF_GEN__REF_POSE_HPP_
#define PR_REF_GEN__REF_POSE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "pr_msgs/msg/pr_array_h.hpp"
#include "std_msgs/msg/bool.hpp"

#include "eigen3/Eigen/Dense"

namespace pr_ref_gen
{
    class RefPose : public rclcpp::Node
    {
        public:
            //PR_REF_GEN_PUBLIC
            explicit RefPose(const rclcpp::NodeOptions & options);

        protected:
            void topic_callback(const pr_msgs::msg::PRArrayH::SharedPtr q_msg);

        private:
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr subscription_;
            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_end_;
            std::vector<double> robot_params;
            std::string ref_path;
            bool is_cart;
            Eigen::MatrixXd ref_matrix;
            int n_ref;
            int idx=0;
    };
}

#endif // PR_REF_GEN__REF_POSE_HPP_