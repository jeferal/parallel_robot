#ifndef PR_MODELLING_RAST_T_HPP_
#define PR_MODELLING_RAST_T_HPP_

#include "rclcpp/rclcpp.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"

#include "pr_msgs/msg/pr_array_h.hpp"
#include "pr_msgs/msg/pr_mat_h.hpp"
#include "pr_lib/pr_model.hpp"
#include "pr_lib/pr_utils.hpp"

#include "eigen3/Eigen/Dense"

namespace pr_modelling
{
    class RastT : public rclcpp::Node
    {
        public:
            //PR_MODELLING_PUBLIC
            explicit RastT(const rclcpp::NodeOptions & options);

        protected:
            void topic_callback(const pr_msgs::msg::PRMatH::ConstPtr& dep_jac_msg,
                                const pr_msgs::msg::PRMatH::ConstPtr& ind_jac_msg);

        private:
            message_filters::Subscriber<pr_msgs::msg::PRMatH> sub_dep;
            message_filters::Subscriber<pr_msgs::msg::PRMatH> sub_ind;

            /*typedef message_filters::sync_policies::ApproximateTime
                    <pr_msgs::msg::PRMatH, pr_msgs::msg::PRMatH> SyncPolicy;*/

            typedef message_filters::sync_policies::ExactTime
                    <pr_msgs::msg::PRMatH, pr_msgs::msg::PRMatH> SyncPolicy;

            typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
            std::shared_ptr<Synchronizer> sync_;

            rclcpp::Publisher<pr_msgs::msg::PRMatH>::SharedPtr publisher_;

            Eigen::Matrix<double, 11, 11> DepJ;
            Eigen::Matrix<double, 11, 4> IndJ;
            Eigen::Matrix<double, 15, 4> Rast;
    };
}

#endif // PR_MODELLING_RAST_T_HPP_