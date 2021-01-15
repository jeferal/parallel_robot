#include "pr_ref_gen/ref_server.hpp"

#include <chrono>
#include <memory>
#include <utility>
#include <array>
#include <vector>
#include <string>
#include <cmath>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "pr_msgs/msg/pr_array_h.hpp"
//#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/bool.hpp"

#include "pr_lib/pr_model.hpp"
#include "pr_lib/pr_utils.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

using namespace std::chrono_literals;


namespace pr_ref_gen
{
    /**** REFERENCE SERVER GENERATOR COMPONENT ****/
    RefServer::RefServer(const rclcpp::NodeOptions & options)
    : Node("ref_server", options)
    {
        RCLCPP_INFO(this->get_logger(), "Starting reference service node");

        this->declare_parameter<std::vector<double>>("robot_config_params", 
            {0.4, 0.4, 0.4, 0.15, 90*(M_PI/180), 45*(M_PI/180), 0.3, 0.3, 0.3, 50*(M_PI/180), 90*(M_PI/180)});

        this->get_parameter("robot_config_params", robot_params);

        //Create communication
        publisher_ = this->create_publisher<pr_msgs::msg::PRArrayH>(
            "ref_pose",
            1
        );

        subscription_ = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "joint_position",
            1,
            std::bind(&RefServer::topic_callback, this, _1)
        );

        service_ = this->create_service<pr_msgs::srv::Trajectory>(
            "perform_trajectory", 
            std::bind(&RefServer::server_callback, this, _1, _2)
        );

        publisher_running_ = create_publisher<std_msgs::msg::Bool>(
            "is_running",
            1
        );

        //client = this->create_client<std_srvs::srv::SetBool>("set_break");

        //int err = set_break_srv(true);

        RCLCPP_INFO(this->get_logger(), "Ready to perfom trajectory");
    }

    void RefServer::server_callback(const pr_msgs::srv::Trajectory::Request::SharedPtr request,
                                    pr_msgs::srv::Trajectory::Response::SharedPtr response)
    {
        //Trajectory already running
        if(is_running){
            RCLCPP_ERROR(this->get_logger(), "Trajectory already running");
            response->error_code = 1;
            response->n_ref_loaded = 0;
            return;
        }
        
        //Read file
        if(PRUtils::read_file(ref_matrix, request->path_trajectory)==-1){
            RCLCPP_ERROR(this->get_logger(), "Could not open file");
            response->error_code = 2;
            response->n_ref_loaded = 0;
            return;
        }

        n_ref = ref_matrix.rows();

        RCLCPP_INFO(this->get_logger(), "Pose references file opened, contains %d references", n_ref);

        if(request->is_cart)
        {
            //Cartesian to joint conversion (Inverse kinematics)
            for(int i=0; i<n_ref; i++)
            {
                Eigen::RowVector4d q_row;
                Eigen::RowVector4d x_row = ref_matrix.row(i);
                PRModel::InverseKinematicsPrism(q_row, x_row, robot_params);
                ref_matrix.row(i) = q_row; 
            }
        }

        RCLCPP_INFO(this->get_logger(), "Trajectory loaded, disabling break");

        //int err = set_break_srv(false);
        /*
        int err = 0;
        if(err == -2){
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
            return;
        }
        */
        /*
        if(err == -1){
            RCLCPP_ERROR(this->get_logger(), "Not success in break service call");
            response->error_code = 3;
            response->n_ref_loaded = n_ref;
        }
        */

        is_running = true;

    }

    void RefServer::topic_callback(const pr_msgs::msg::PRArrayH::SharedPtr q_msg)
    {
        //RCLCPP_INFO(this->get_logger(), "topic callback");
        if(is_running) {
            
            auto ref_msg = pr_msgs::msg::PRArrayH();
            //CONVERTIR A FUNCIÓN
            for(int i=0; i<4; i++)
                ref_msg.data[i] = ref_matrix(idx, i);

            ref_msg.current_time = this->get_clock()->now();
            ref_msg.header.stamp = q_msg->header.stamp;
            ref_msg.header.frame_id = q_msg->header.frame_id;
            //RCLCPP_INFO(this->get_logger(), "msg.data[0] = %f, idx: %d", ref_msg.data[0], idx);
            publisher_->publish(ref_msg);

            if(idx < n_ref - 1){
                idx++;
            } else {
                is_running=false;
            }
        }

        auto status_msg = std_msgs::msg::Bool();

        status_msg.data = is_running;

        publisher_running_->publish(status_msg);
    }

/*
    int RefServer::set_break_srv(const bool data)
    {
        auto request_break = std::make_shared<std_srvs::srv::SetBool::Request>();
        request_break->data = data;

        //Gestionar bien errores!!
        while (!client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                return -2;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }

        auto response_break = client->async_send_request(request_break);

        //Wait for response

        if(!response_break.get()->success) {
            return -1;
        }

        RCLCPP_INFO(this->get_logger(), "Break status: %s", response_break.get()->message.c_str());
        return 0;
    }
*/
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_ref_gen::RefServer)