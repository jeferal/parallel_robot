#ifndef PR_REF_GEN__SING_EVADER_HPP_
#define PR_REF_GEN__SING_EVADER_HPP_

#include "rclcpp/rclcpp.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"

#include "pr_msgs/msg/pr_array_h.hpp"
#include "pr_msgs/msg/prots.hpp"

#include "pr_lib/pr_singularity.hpp"


namespace pr_ref_gen
{
    class SingEvader : public rclcpp::Node
    {
        public:
            //PR_REF_GEN_PUBLIC
            explicit SingEvader(const rclcpp::NodeOptions & options);

        protected:

            void topic_callback(const pr_msgs::msg::PRArrayH::ConstPtr& ref_msg,
                                     const pr_msgs::msg::PRArrayH::ConstPtr& x_msg,
                                     const pr_msgs::msg::PROTS::ConstPtr& ots_msg);

        private:
            message_filters::Subscriber<pr_msgs::msg::PRArrayH> sub_ref;
            message_filters::Subscriber<pr_msgs::msg::PRArrayH> sub_x;
            message_filters::Subscriber<pr_msgs::msg::PROTS> sub_ots;

            typedef message_filters::sync_policies::ExactTime
                    <pr_msgs::msg::PRArrayH, pr_msgs::msg::PRArrayH, 
                     pr_msgs::msg::PROTS> SyncPolicy;

            typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
            std::shared_ptr<Synchronizer> sync_;

            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_;

    };
}

#endif // PR_REF_GEN__SING_EVADER_HPP_