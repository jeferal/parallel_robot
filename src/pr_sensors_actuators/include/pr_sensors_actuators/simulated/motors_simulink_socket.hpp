#ifndef PR_SENSORS_ACTUATORS__MOTORS_SIMULINK_SOCKET_HPP_
#define PR_SENSORS_ACTUATORS__MOTORS_SIMULINK_SOCKET_HPP_

#include "rclcpp/rclcpp.hpp"

#include <vector>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <unistd.h>
#include <string.h>

#include "pr_msgs/msg/pr_array_h.hpp"

#define PORT 5021


namespace pr_sensors_actuators
{

    class MotorsSimulinkSocket : public rclcpp::Node
    {
        public:
            //PR_SENSORS_ACTUATORS_PUBLIC
            explicit MotorsSimulinkSocket(const rclcpp::NodeOptions & options);
            ~MotorsSimulinkSocket();

        protected:

            void topic_callback(const pr_msgs::msg::PRArrayH::SharedPtr control_action_msg);
            void sat_ca(double &control_action, const double &sat);

        private:

            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr subscription_;
            double max_v;
            double volts[4];
            std::vector<double> vp_conversion;
            int new_socket;


    };

}   // Namespace pr_sensors_actuators


#endif // PR_SENSORS_ACTUATORS__ENCODERS_SIMULINK_HPP_