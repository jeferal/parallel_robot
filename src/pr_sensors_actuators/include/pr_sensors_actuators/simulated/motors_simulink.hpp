#ifndef PR_SENSORS_ACTUATORS__MOTORS_SIMULINK_HPP_
#define PR_SENSORS_ACTUATORS__MOTORS_SIMULINK_HPP_

#include "rclcpp/rclcpp.hpp"
#include <vector>

#include "pr_msgs/msg/pr_array_h.hpp"
#include "geometry_msgs/msg/quaternion.hpp"


namespace pr_sensors_actuators
{

    class MotorsSimulink : public rclcpp::Node
    {
        public:
            //PR_SENSORS_ACTUATORS_PUBLIC
            explicit MotorsSimulink(const rclcpp::NodeOptions & options);
            ~MotorsSimulink();

        protected:

            void topic_callback(const pr_msgs::msg::PRArrayH::SharedPtr control_action_msg);
            void sat_ca(double &control_action, const double &sat);

        private:

            rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr publisher_;
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr subscription_;
            double max_v;
            std::vector<double> volts, vp_conversion;


    };

}   // Namespace pr_sensors_actuators


#endif // PR_SENSORS_ACTUATORS__ENCODERS_SIMULINK_HPP_