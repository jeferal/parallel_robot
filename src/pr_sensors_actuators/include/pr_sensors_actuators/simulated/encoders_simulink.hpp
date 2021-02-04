#ifndef PR_SENSORS_ACTUATORS__ENCODERS_SIMULINK_HPP_
#define PR_SENSORS_ACTUATORS__ENCODERS_SIMULINK_HPP_

#include "rclcpp/rclcpp.hpp"
#include <vector>

#include "pr_msgs/msg/pr_array_h.hpp"
#include "geometry_msgs/msg/quaternion.hpp"


namespace pr_sensors_actuators
{

    class EncodersSimulink : public rclcpp::Node
    {
        public:
            //PR_SENSORS_ACTUATORS_PUBLIC
            explicit EncodersSimulink(const rclcpp::NodeOptions & options);

        protected:

            void topic_callback(const geometry_msgs::msg::Quaternion::SharedPtr msg);
            void on_timer();

        private:

            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_;
            rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr subscription_;
            rclcpp::TimerBase::SharedPtr timer_;

            pr_msgs::msg::PRArrayH position_msg;
            std::vector<double> initial_position;
            float ts;
            int iter=0;
            bool is_connected=false;

    };

}   // Namespace pr_sensors_actuators


#endif // PR_SENSORS_ACTUATORS__ENCODERS_SIMULINK_HPP_