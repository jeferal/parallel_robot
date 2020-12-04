#ifndef PR_SENSORS_ACTUATORS__ENCODERS_COMPONENT_HPP_
#define PR_SENSORS_ACTUATORS__ENCODERS_COMPONENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "pr_msgs/msg/pr_array_h.hpp"
#include "std_msgs/msg/bool.hpp"

#include "bdaqctrl.h"
#define deviceDescription L"PCI-1784,BID#0"
using namespace Automation::BDaq;


namespace pr_sensors_actuators
{

    class Encoders : public rclcpp::Node
    {
        public:
            //PR_SENSORS_ACTUATORS_PUBLIC
            explicit Encoders(const rclcpp::NodeOptions & options);

                    ~Encoders();

        protected:

            void on_timer();

            void end_callback(const std_msgs::msg::Bool::SharedPtr end_msg);

        private:

            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_;
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_end_;
            rclcpp::TimerBase::SharedPtr timer_;

            //Encoder instances
            UdCounterCtrl* udCounterCtrl0;
		    UdCounterCtrl* udCounterCtrl1;
		    UdCounterCtrl* udCounterCtrl2;
		    UdCounterCtrl* udCounterCtrl3;
            

            //Brakes
            InstantDoCtrl* instantDoCtrl;
		    uint8 DOut[1];

		    ErrorCode ret;

            long long int pulsos[4];
            double position[4];
            float ts;
            bool is_finished = false;
            std::vector<double> initial_position;

    };

}   // Namespace pr_sensors_actuators


#endif // PR_SENSORS_ACTUATORS__ENCODERS_COMPONENT_HPP_