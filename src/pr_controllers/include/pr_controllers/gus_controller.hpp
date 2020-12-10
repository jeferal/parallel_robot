#ifndef PR_CONTROLLERS__GUS_CONTROLLER_HPP_
#define PR_CONTROLLERS__GUS_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include "eigen3/Eigen/Dense"

#include "pr_msgs/msg/pr_array_h.hpp"


namespace pr_controllers
{
    class GusController : public rclcpp::Node
    {
        public:
            //PR_COONTROLLERS_PUBLIC
            explicit GusController(const rclcpp::NodeOptions & options);

        protected:

            void controller_callback(const pr_msgs::msg::PRArrayH::ConstPtr& ref_msg,
                                     const pr_msgs::msg::PRArrayH::ConstPtr& pos_msg,
                                     const pr_msgs::msg::PRArrayH::ConstPtr& vel_msg);

        private:
            message_filters::Subscriber<pr_msgs::msg::PRArrayH> sub_ref;
            message_filters::Subscriber<pr_msgs::msg::PRArrayH> sub_pos;
            message_filters::Subscriber<pr_msgs::msg::PRArrayH> sub_vel;

            typedef message_filters::sync_policies::ApproximateTime
                    <pr_msgs::msg::PRArrayH, pr_msgs::msg::PRArrayH, pr_msgs::msg::PRArrayH> SyncPolicy;

            typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
            std::shared_ptr<Synchronizer> sync_;

            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_;

            double ts, k1, k2;
            Eigen::RowVector4d ref_ant, q_ant, up_1_ant;

    };
}

#endif // PR_CONTROLLERS__GUS_CONTROLLER_HPP_