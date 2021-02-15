#ifndef PR_SENSORS_ACTUATORS__SIMULINK_SOCKET_INTERFACE_HPP_
#define PR_SENSORS_ACTUATORS__SIMULINK_SOCKET_INTERFACE_HPP_

#include <unistd.h> 
#include <stdio.h> 
#include <sys/socket.h> 
#include <stdlib.h> 
#include <netinet/in.h> 
#include <string.h>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "pr_msgs/msg/pr_array_h.hpp"


namespace pr_sensors_actuators
{
    class SimulinkSocketInterface : public rclcpp::Node
    {
        public:
            //PR_SENSORS_ACTUATORS_PUBLIC
            explicit SimulinkSocketInterface(const rclcpp::NodeOptions & options);

        protected:
            void topic_callback(const pr_msgs::msg::PRArrayH::SharedPtr control_action_msg);
            void on_timer();
            void sat_ca(double &control_action, const double &sat);

        private:
            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_;
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr subscription_;
            rclcpp::TimerBase::SharedPtr timer_;

            rclcpp::callback_group::CallbackGroup::SharedPtr cb_group_timer;
            rclcpp::callback_group::CallbackGroup::SharedPtr cb_group_sub;

            int server_fd, new_socket;
            std::vector<double> vp_conversion;
            double max_v;
            pr_msgs::msg::PRArrayH pos_msg;
            
    };

}   // Namespace pr_sensors_actuators

#endif // PR_SENSORS_ACTUATORS__SIMULINK_SOCKET_INTERFACE_HPP_