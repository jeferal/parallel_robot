#ifndef PR_REF_GEN__REF_SERVER_HPP_
#define PR_REF_GEN__REF_SERVER_HPP_

#include "rclcpp/rclcpp.hpp"

#include "pr_msgs/msg/pr_array_h.hpp"
#include "pr_msgs/srv/trajectory.hpp"

#include "std_msgs/msg/bool.hpp"
//#include "std_srvs/srv/set_bool.hpp"

#include "eigen3/Eigen/Dense"

namespace pr_ref_gen
{
    class RefServer : public rclcpp::Node
    {
        public:
            //PR_REF_SERVER_PUBLIC
            explicit RefServer(const rclcpp::NodeOptions & options);

        protected:
            void topic_callback(const pr_msgs::msg::PRArrayH::SharedPtr q_msg);
            void server_callback(const pr_msgs::srv::Trajectory::Request::SharedPtr request,
                                 const pr_msgs::srv::Trajectory::Response::SharedPtr response);


            //int set_break_srv(const bool data);

        private:
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr subscription_;
            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_running_;
            rclcpp::Service<pr_msgs::srv::Trajectory>::SharedPtr service_;

            //rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client;

            //rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_end_;
            std::vector<double> robot_params;
            std::string ref_path;
            Eigen::MatrixXd ref_matrix;
            int n_ref;
            int idx=0;
            bool is_cart, is_running=false;
    };
}

#endif // PR_REF_GEN__REF_SERVER_HPP_