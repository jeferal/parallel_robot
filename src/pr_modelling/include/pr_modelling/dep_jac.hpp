#ifndef PR_MODELLING__JAC_DEP_HPP_
#define PR_MODELLING__JAC_DEP_HPP_

#include "rclcpp/rclcpp.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include "pr_msgs/msg/pr_array_h.hpp"
#include "pr_msgs/msg/pr_mat_h.hpp"
#include "pr_lib/pr_model.hpp"
#include "pr_lib/pr_utils.hpp"

#include "eigen3/Eigen/Dense"

namespace pr_modelling
{
    class DependentJacobian : public rclcpp::Node
    {
        public:
            //PR_MODELLING_PUBLIC
            explicit DependentJacobian(const rclcpp::NodeOptions & options);

        protected:
            void topic_callback(const pr_msgs::msg::PRArrayH::ConstPtr& x_coord_msg,
                                const pr_msgs::msg::PRMatH::ConstPtr& q_msg);

        private:
            message_filters::Subscriber<pr_msgs::msg::PRArrayH> sub_x;
            message_filters::Subscriber<pr_msgs::msg::PRMatH> sub_q;

            typedef message_filters::sync_policies::ApproximateTime
                    <pr_msgs::msg::PRArrayH, pr_msgs::msg::PRMatH> SyncPolicy;

            typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
            std::shared_ptr<Synchronizer> sync_;

            rclcpp::Publisher<pr_msgs::msg::PRMatH>::SharedPtr publisher_;

            std::vector<double> robot_params;
            Eigen::Matrix<double, 11, 11> DepJ;
            Eigen::Matrix<double, 4, 3> Q;
    };
}

#endif // PR_MODELLING__JAC_DEP_HPP_