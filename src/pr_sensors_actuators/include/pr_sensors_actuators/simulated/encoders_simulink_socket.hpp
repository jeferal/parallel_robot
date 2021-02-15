#ifndef PR_SENSORS_ACTUATORS__ENCODERS_SIMULINK_SOCKET_HPP_
#define PR_SENSORS_ACTUATORS__ENCODERS_SIMULINK_SOCKET_HPP_

#include "rclcpp/rclcpp.hpp"
#include <vector>

#include "pr_msgs/msg/pr_array_h.hpp"

#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <unistd.h>
#include <string.h>

#define PORT 5021

namespace pr_sensors_actuators
{

    class EncodersSimulinkSocket : public rclcpp::Node
    {
        public:
            //PR_SENSORS_ACTUATORS_PUBLIC
            explicit EncodersSimulinkSocket(const rclcpp::NodeOptions & options);

        protected:

            void on_timer();

        private:

            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_;
            rclcpp::TimerBase::SharedPtr timer_;

            std::vector<double> initial_position;
            float ts;
            int iter=0;
            bool is_connected=false;

            int new_socket;

    };

}   // Namespace pr_sensors_actuators


#endif // PR_SENSORS_ACTUATORS__ENCODERS_SIMULINK_SOCKET_HPP_