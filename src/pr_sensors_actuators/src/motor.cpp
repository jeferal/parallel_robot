#include "pr_sensors_actuators/motor.hpp"

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "pr_msgs/msg/pr_array_h.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace Automation::BDaq;
using std::placeholders::_1;


namespace pr_sensors_actuators
{
    /**** ENCODER COMPONENT ****/
    Motor::Motor(const rclcpp::NodeOptions & options)
    : Node("motor_3", options)
    {
        //Parameter declaration
        this->declare_parameter<std::vector<double>>("vp_conversion",{1.0, 1.0, 1.0, 1.0});
        this->get_parameter("vp_conversion", vp_conversion);
        
        //Get motor number from node name
        const char *pn;
        pn = this->get_name();
        pn = pn + strlen(this->get_name())-1;
        n_motor = atoi(pn);

        if(n_motor>4 || n_motor<0){
            RCLCPP_ERROR(this->get_logger(), "Wrong motor number");
            //Find a better way to exit
            exit(1);
        }

        RCLCPP_INFO(this->get_logger(), "This is actuator number %d", n_motor);

        //Create communication
        subscription_=this->create_subscription<pr_msgs::msg::PRArrayH>(
            "control_action",
                        1,
            std::bind(&Motor::topic_callback,this,_1));

        subscription_end_= this->create_subscription<std_msgs::msg::Bool>(
            "end_flag", 
                     1, 
            std::bind(&Motor::end_callback, this, _1));

        //Initialize actuator
        init_ao_pci();
        is_finished = false;
    
    }

    Motor::~Motor()
    {
        //Writing 0
        volts=0.0;
        pci1720->Write(n_motor, 1 ,&volts);
        pci1720->Dispose();
        RCLCPP_INFO(this->get_logger(), "Shutting down actuator number %d", n_motor);			
	}

    void Motor::topic_callback(const pr_msgs::msg::PRArrayH::SharedPtr control_action_msg){
        if(is_finished == false){
            RCLCPP_INFO(this->get_logger(), "I heard: '%f'", control_action_msg->data[n_motor]);
               
            volts = control_action_msg->data[n_motor]/vp_conversion[n_motor];

            //Control action saturation
            sat_ca(volts, 4.0);
               
            RCLCPP_INFO(this->get_logger(), "Control action %f", volts);
               
            pci1720->Write(n_motor, 1, &volts);
        }
    }

    void Motor::end_callback(const std_msgs::msg::Bool::SharedPtr end_msg){
          if(end_msg->data == true){
               is_finished = true;
               volts = 0;
               pci1720->Write(n_motor, 1, &volts);
               RCLCPP_INFO(this->get_logger(), "Finished, written: %f", volts);
          }
    }

    void Motor::init_ao_pci(void){
          ret = Success;

          pci1720=AdxInstantAoCtrlCreate();
          DeviceInformation devInfo(deviceDescription);
          ret = pci1720->setSelectedDevice(devInfo);
          
          RCLCPP_INFO(this->get_logger(), "Actuator %d initialised: %d", n_motor, ret);
          volts = 0.0;
          ret = pci1720->Write(n_motor, 1, &volts);
    }

    void saturar_accion_control(double &control_action, const double sat = 7.0){
	     if(control_action > sat) control_action = sat;
	     if(control_action < -sat) control_action = -sat;
    }

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_sensors_actuators::Motor)