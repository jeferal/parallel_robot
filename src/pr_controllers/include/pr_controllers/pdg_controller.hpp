#ifndef PR_CONTROLLERS__PDG_CONTROLLER_HPP_
#define PR_CONTROLLERS__PDG_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include "pr_msgs/msg/pr_array_h.hpp"


namespace pr_controllers
{
    class PDGController : public rclcpp::Node
    {
        public:
            //PR_COONTROLLERS_PUBLIC
            explicit PDGController(const rclcpp::NodeOptions & options);

        protected:

            void controller_callback(const pr_msgs::msg::PRArrayH::ConstPtr& ref_msg,
                                     const pr_msgs::msg::PRArrayH::ConstPtr& pos_msg,
                                     const pr_msgs::msg::PRArrayH::ConstPtr& vel_msg,
                                     const pr_msgs::msg::PRArrayH::ConstPtr& grav_msg);

        private:
            message_filters::Subscriber<pr_msgs::msg::PRArrayH> sub_ref;
            message_filters::Subscriber<pr_msgs::msg::PRArrayH> sub_pos;
            message_filters::Subscriber<pr_msgs::msg::PRArrayH> sub_vel;
            message_filters::Subscriber<pr_msgs::msg::PRArrayH> sub_grav;

            typedef message_filters::sync_policies::ApproximateTime
                    <pr_msgs::msg::PRArrayH, pr_msgs::msg::PRArrayH, 
                     pr_msgs::msg::PRArrayH, pr_msgs::msg::PRArrayH> SyncPolicy;

            typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
            std::shared_ptr<Synchronizer> sync_;

            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_;

            std::vector<double> Kp, Kv;
    };
}

#endif // PR_CONTROLLERS__PDG_CONTROLLER_HPP_