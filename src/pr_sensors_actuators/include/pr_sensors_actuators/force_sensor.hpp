#ifndef PR_SENSORS_ACTUATORS__FORCE_SENSOR_HPP_
#define PR_SENSORS_ACTUATORS__FORCE_SENSOR_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <cstdint>
#include <unistd.h>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "pr_msgs/msg/pr_array.hpp"
#include "pr_msgs/msg/pr_force_state.hpp"

#define PORT 49152 /* Port the Net F/T always uses */
#define COMMAND 2 /* Command code 2 starts streaming */
#define NUM_SAMPLES 1 /* Will send 1 sample before stopping */


///***** TODO ******///
namespace pr_sensors_actuators
{
    class ForceSensor : public rclcpp::Node
    {
        public:
            //PR_SENSORS_ACTUATORS_PUBLIC
            explicit ForceSensor(const rclcpp::NodeOptions & options);

        protected:
            void 
    };

}   // Namespace pr_sensors_actuators


#endif // PR_SENSORS_ACTUATORS__FORCE_SENSOR_HPP_