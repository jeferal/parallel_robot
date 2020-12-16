#ifndef PR_MODELLING_Q_GRAV_HPP_
#define PR_MODELLING_Q_GRAV_HPP_

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
    class QGrav : public rclcpp::Node
    {
        public:
            //PR_MODELLING_PUBLIC
            explicit QGrav(const rclcpp::NodeOptions & options);

        protected:
            void topic_callback(const pr_msgs::msg::PRArrayH::ConstPtr& x_msg,
                                const pr_msgs::msg::PRMatH::ConstPtr& q_msg,
                                const pr_msgs::msg::PRMatH::ConstPtr& rast_t_msg);

        private:
            message_filters::Subscriber<pr_msgs::msg::PRArrayH> sub_x;
            message_filters::Subscriber<pr_msgs::msg::PRMatH> sub_q;
            message_filters::Subscriber<pr_msgs::msg::PRMatH> sub_rast_t;

            /*typedef message_filters::sync_policies::ApproximateTime
                    <pr_msgs::msg::PRArrayH, pr_msgs::msg::PRMatH, pr_msgs::msg::PRMatH> SyncPolicy;
            */
            typedef message_filters::sync_policies::ExactTime
                    <pr_msgs::msg::PRArrayH, pr_msgs::msg::PRMatH, pr_msgs::msg::PRMatH> SyncPolicy;
            
            typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
            std::shared_ptr<Synchronizer> sync_;

            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_;

            std::vector<double> robot_params;
            std::vector<double> P11, P12, P21, P22, P31, P32, P41, P42, Pm;
            Eigen::Matrix<double, 4, 3> Q;
            Eigen::Matrix<double, 4, 15> RastT;
            Eigen::Vector4d QGravTerms;
    };
}

#endif // PR_MODELLING_RAST_T_HPP_