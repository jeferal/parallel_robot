#ifndef PR_SENSORS_ACTUATORS__ENCODERS_SIM_HPP_
#define PR_SENSORS_ACTUATORS__ENCODERS_SIM_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "pr_msgs/msg/pr_array_h.hpp"


namespace pr_sensors_actuators
{

    class EncodersSim : public rclcpp::Node
    {
        public:
            //PR_SENSORS_ACTUATORS_PUBLIC
            explicit EncodersSim(const rclcpp::NodeOptions & options);

        protected:

            void on_timer();

        private:

            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_;
            rclcpp::TimerBase::SharedPtr timer_;

    };

}   // Namespace pr_sensors_actuators


#endif // PR_SENSORS_ACTUATORS__ENCODERS_SIM_HPP_