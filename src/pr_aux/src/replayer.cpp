#include "pr_aux/replayer.hpp"

#include <chrono>
#include <memory>
#include <utility>
#include <array>
#include <vector>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "pr_msgs/msg/pr_array_h.hpp"

#include "pr_lib/pr_model.hpp"
#include "pr_lib/pr_utils.hpp"

using std::placeholders::_1;

namespace pr_aux
{
    /**** REPLAYER DATA GENERATOR COMPONENT ****/
    Replayer::Replayer(const rclcpp::NodeOptions & options)
    : Node("replayer", options)
    {
        //Parameter declaration
        this->declare_parameter<float>("ts_ms", 10.0);
        this->declare_parameter<std::string>("data_path", 
            "/home/paralelo4dofnew/parallel_robot_ws/references/ref_cart_TRR0_CF1_IdV1.txt");
        
        this->get_parameter("ts_ms", ts_ms);
        this->get_parameter("data_path", data_path);

        //Read file
        std::cout << "About to read the file" << std::endl;
        if(PRUtils::read_file(data_matrix, data_path)==-1){
            RCLCPP_ERROR(this->get_logger(), "Could not open file");
            std::cout << "Could not read the file" << std::endl;
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Data file file opened");
        n_data = data_matrix.rows();

        //Create communication
        publisher_ = this->create_publisher<pr_msgs::msg::PRArrayH>(
            "data",
            1
        );

        //End signal subscription
        subscription_end_ = this->create_subscription<std_msgs::msg::Bool>(
            "end_flag",
            1,
            std::bind(&Replayer::end_callback, this, _1));

        //Create timer
        timer_ = this->create_wall_timer(
            std::chrono::duration<float, std::milli>(ts_ms), 
            std::bind(&Replayer::on_timer, this));

        
    }

    void Replayer::on_timer()
    {
        if(!is_finished) {
        
            auto data_msg = pr_msgs::msg::PRArrayH();
            //CONVERTIR A FUNCIÓN
            for(int i=0; i<4; i++)
                data_msg.data[i] = data_matrix(idx, i);

            data_msg.current_time = this->get_clock()->now();
            data_msg.header.stamp = data_msg.current_time;
            data_msg.header.frame_id = std::to_string(idx);
            publisher_->publish(data_msg); 
            
            if(idx<n_data-1)
                idx++;

        } else {
            //End flag activted
			RCLCPP_INFO(this->get_logger(), "Experiment finished");
		}
	}

    void Replayer::end_callback(const std_msgs::msg::Bool::SharedPtr end_msg)
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
RCLCPP_COMPONENTS_REGISTER_NODE(pr_aux::Replayer)