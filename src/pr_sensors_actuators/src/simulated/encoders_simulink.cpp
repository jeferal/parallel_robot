#include "pr_sensors_actuators/simulated/encoders_simulink.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "pr_msgs/msg/pr_array_h.hpp"

#include "rclcpp/qos.hpp"

using std::placeholders::_1;


namespace pr_sensors_actuators
{
    /**** ENCODERS SIMULINK COMPONENT ****/
    
    EncodersSimulink::EncodersSimulink(const rclcpp::NodeOptions & options)
    : Node("position_sensors", options)
    {
        //Parameter declaration
        this->declare_parameter<float>("ts_ms", 10.0);
        this->declare_parameter<std::vector<double>>("initial_position", {0.665559, 0.654974, 0.691026, 0.631511});


        //Read parameters
        this->get_parameter("ts_ms", ts);
        this->get_parameter("initial_position", initial_position);

        //Position publisher
        publisher_ = this->create_publisher<pr_msgs::msg::PRArrayH>(
			"joint_position", 
			1);

        rclcpp::SensorDataQoS sensor_qos;
        sensor_qos.keep_last(1);

        subscription_ = this->create_subscription<geometry_msgs::msg::Quaternion>(
            "posicion_sim",
            sensor_qos,
            std::bind(&EncodersSimulink::topic_callback,this,_1)
        );

        //Create timer
        /*
        timer_ = this->create_wall_timer(
            std::chrono::duration<float, std::milli>(ts), 
            std::bind(&EncodersSimulink::on_timer, this));

        position_msg.data[0] = initial_position[0];
        position_msg.data[1] = initial_position[1];
        position_msg.data[2] = initial_position[2];
        position_msg.data[3] = initial_position[3];

        position_msg.current_time = this->get_clock()->now();
		position_msg.header.stamp = position_msg.current_time;

        publisher_->publish(position_msg);
        */
    }

    void EncodersSimulink::topic_callback(const geometry_msgs::msg::Quaternion::SharedPtr msg) {
        /*if(msg->x==0.0){
            position_msg.data[0] = 0.656465;
            position_msg.data[1] = 0.647034;
            position_msg.data[2] = 0.677844;
            position_msg.data[3] = 0.619849;
          } else {

          position_msg.data[0] = msg->x;
          position_msg.data[1] = msg->y;
          position_msg.data[2] = msg->z;
          position_msg.data[3] = msg->w;
          }*/
        
        //Pose saturation to avoid problems with RL algorithm
        auto pos_msg = pr_msgs::msg::PRArrayH();

        pos_msg.data[0] = msg->x;
        pos_msg.data[1] = msg->y;
        pos_msg.data[2] = msg->z;
        pos_msg.data[3] = msg->w;

        pos_msg.current_time = this->get_clock()->now();
		pos_msg.header.stamp = this->get_clock()->now();

        publisher_->publish(pos_msg);

    }

    void EncodersSimulink::on_timer() {
        /*
        is_connected = true;
        position_msg.current_time = this->get_clock()->now();
		position_msg.header.stamp = this->get_clock()->now();

        if(is_connected){
          publisher_->publish(position_msg);
          //RCLCPP_INFO(this->get_logger(), "Publishing: %f %f %f %f", position_msg.data[0], position_msg.data[1], position_msg.data[2], position_msg.data[3]);
        }
        */
    }


}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_sensors_actuators::EncodersSimulink)