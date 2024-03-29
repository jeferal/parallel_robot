#ifndef PR_MODELLING__STATE_PUBLISHER_HPP_
#define PR_MODELLING__STATE_PUBLISHER_HPP_

#include <array>

#include "rclcpp/rclcpp.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"

#include "pr_msgs/msg/pr_array_h.hpp"
#include "pr_msgs/msg/pr_state.hpp"


namespace pr_modelling
{
    class StatePublisher : public rclcpp::Node
    {
        public:
            //PR_MODELLING_PUBLIC
            explicit StatePublisher(const rclcpp::NodeOptions & options);

        protected:
            void topic_callback(const pr_msgs::msg::PRArrayH::ConstPtr& q_msg,
                                const pr_msgs::msg::PRArrayH::ConstPtr& q_vel_msg);

            void saturate(std::array<double,4> &var, const double v_min, const double v_max);

        private:
            message_filters::Subscriber<pr_msgs::msg::PRArrayH> sub_q;
            message_filters::Subscriber<pr_msgs::msg::PRArrayH> sub_q_vel;

            /*
            typedef message_filters::sync_policies::ApproximateTime
                    <pr_msgs::msg::PRArrayH, pr_msgs::msg::PRMatH> SyncPolicy;
            */
            typedef message_filters::sync_policies::ExactTime
                    <pr_msgs::msg::PRArrayH, pr_msgs::msg::PRArrayH> SyncPolicy;

            typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
            std::shared_ptr<Synchronizer> sync_;

            rclcpp::Publisher<pr_msgs::msg::PRState>::SharedPtr publisher_;

            double pos_min, pos_max, vel_min, vel_max;
    };
}

#endif // PR_MODELLING__STATE_PUBLISHER_HPP_