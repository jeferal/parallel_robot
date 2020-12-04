#include "pr_sensors_actuators/encoders.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "pr_msgs/msg/pr_array_h.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;
using namespace Automation::BDaq;

using std::placeholders::_1;

namespace pr_sensors_actuators
{
    /**** ENCODER COMPONENT ****/
    
    Encoders::Encoders(const rclcpp::NodeOptions & options)
    : Node("position_sensors", options)
    {
        //Parameter declaration
        this->declare_parameter<float>("ts", 10.0);
        this->declare_parameter<std::vector<double>>("initial_position", {0.665559, 0.654974, 0.691026, 0.631511});

        //Read parameters
        this->get_parameter("ts", ts);
        this->get_parameter("initial_position", initial_position);

        //Position publisher
        publisher_ = this->create_publisher<pr_msgs::msg::PRArrayH>("joint_position", 1);
        //End signal subscription
        subscription_end_ = this->create_subscription<std_msgs::msg::Bool>(
            "end_flag",
            1,
            std::bind(&Encoders::end_callback, this, _1)
        );

        //Create timer
        timer_ = this->create_wall_timer(
            std::chrono::duration<float, std::milli>(ts), 
            std::bind(&Encoders::on_timer, this));

        //Encoder channels initialization
		DeviceInformation devInfo(deviceDescription);
			
		udCounterCtrl0=AdxUdCounterCtrlCreate();
		udCounterCtrl1=AdxUdCounterCtrlCreate();
		udCounterCtrl2=AdxUdCounterCtrlCreate();
		udCounterCtrl3=AdxUdCounterCtrlCreate();
		instantDoCtrl = AdxInstantDoCtrlCreate();

		ret=udCounterCtrl0->setSelectedDevice(devInfo);
		ret=udCounterCtrl1->setSelectedDevice(devInfo);
		ret=udCounterCtrl2->setSelectedDevice(devInfo);
		ret=udCounterCtrl3->setSelectedDevice(devInfo);
		ret=instantDoCtrl->setSelectedDevice(devInfo);


		ret = udCounterCtrl0->setChannel(0);
		RCLCPP_INFO(this->get_logger(), "Channel 0 configuration completed: %d", ret);
		ret = udCounterCtrl1->setChannel(1);
		RCLCPP_INFO(this->get_logger(), "Channel 1 configuration completed: %d", ret);
		ret = udCounterCtrl2->setChannel(2);
		RCLCPP_INFO(this->get_logger(), "Channel 2 configuration completed: %d", ret);
		ret = udCounterCtrl3->setChannel(3);
		RCLCPP_INFO(this->get_logger(), "Channel 3 configuration completed: %d", ret);

		ret=udCounterCtrl0->setCountingType(AbPhaseX1);
		ret=udCounterCtrl1->setCountingType(AbPhaseX1);
		ret=udCounterCtrl2->setCountingType(AbPhaseX1);
		ret=udCounterCtrl3->setCountingType(AbPhaseX1);

		ret=udCounterCtrl0->setEnabled(true);
		ret=udCounterCtrl1->setEnabled(true);
		ret=udCounterCtrl2->setEnabled(true);
		ret=udCounterCtrl3->setEnabled(true);

        RCLCPP_INFO(this->get_logger(), "Brake disabled");

			ret = instantDoCtrl->Read(0,1,DOut);

			for (int i=0; i<4; i++){
				DOut[0] = DOut[0]|(0x01<<i);
			}

			//std::cout << DOut[0] << std::endl;

			ret = instantDoCtrl->Write(0,1,DOut);

			RCLCPP_INFO(this->get_logger(), "Configuration completed, brake disabled");
    }

    Encoders::~Encoders()
    {
			
		DOut[0]=0;
	    ret = instantDoCtrl->Write(0,1,DOut);

		udCounterCtrl0->Dispose();
		udCounterCtrl1->Dispose();
		udCounterCtrl2->Dispose();
		udCounterCtrl3->Dispose();

	    instantDoCtrl->Dispose();
		RCLCPP_INFO(this->get_logger(), "Encoders node finished");
	}

    void Encoders::on_timer()
    {
		if(!is_finished)
        {
			auto position_msg = pr_msgs::msg::PRArrayH();

			//First joint
			pulsos[0] = udCounterCtrl0->getValue();
			position_msg.data[0] = pulsos[0]*0.00002 + initial_position[0];

			//Second joint
			pulsos[1] = udCounterCtrl1->getValue();
			position_msg.data[1] = pulsos[1]*0.00002 + initial_position[1];

			//Third joint
			pulsos[2] = udCounterCtrl2->getValue();
			position_msg.data[2] = pulsos[2]*0.00002 + initial_position[2];

			//Fourth joint
			pulsos[3] = udCounterCtrl3->getValue();
			position_msg.data[3] = pulsos[3]*0.000002325 + initial_position[3];

			//Time clock
			position_msg.header.stamp = this->get_clock()->now();	    

			publisher_->publish(position_msg);
            
            
			RCLCPP_INFO(this->get_logger(), "Publicando: %f %f %f %f", 
            position_msg.data[0], 
            position_msg.data[1], 
            position_msg.data[2], 
            position_msg.data[3]);
            
		} 
        else 
        {
            //End flag activted
			DOut[0]=0;
	    	ret = instantDoCtrl->Write(0,1,DOut);
			RCLCPP_INFO(this->get_logger(), "Freno activado");
		}
	}

    void Encoders::end_callback(const std_msgs::msg::Bool::SharedPtr end_msg)
    {
        //End callback looking for the final event
        if(end_msg->data)
			is_finished = true;
    }

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_sensors_actuators::Encoders)