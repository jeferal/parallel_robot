#include "pr_ref_gen/sing_evader.hpp"

#include <chrono>
#include <memory>
#include <utility>
#include <array>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "pr_msgs/msg/pr_array_h.hpp"
#include "pr_lib/pr_utils.hpp"


namespace pr_ref_gen
{
    /**** Singularity CONTROLLER COMPONENT ****/
    SingEvader::SingEvader(const rclcpp::NodeOptions & options)
    : Node("sin_evader", options)
    {
        //Parameter declaration
        this->declare_parameter<std::vector<double>>(
            "robot_config_params", 
            {0.4, 0.4, 0.4, 0.15, 90*(M_PI/180), 45*(M_PI/180), 0.3, 0.3, 0.3, 50*(M_PI/180), 90*(M_PI/180)});
        this->declare_parameter<int>("iter_max",30);
        this->declare_parameter<double>("tol",1e-7);
        this->declare_parameter<double>("tol_OTS",1e-7);
        this->declare_parameter<int>("iter_OTS",30);
        this->declare_parameter<double>("t_activation",5);
        this->declare_parameter<int>("ncomb",4);
        this->declare_parameter<double>("ts", 0.01);
        this->declare_parameter<double>("lmin_Ang_OTS",3.0);

        this->get_parameter("robot_config_params", robot_params);
        this->get_parameter("iter_max", iter_max);
        this->get_parameter("tol", tol);
        this->get_parameter("tol_OTS", tol_OTS);
        this->get_parameter("iter_OTS", iter_OTS);
        this->get_parameter("t_activation", t_activation);
        this->get_parameter("ncomb",ncomb);
        this->get_parameter("ts", ts);
        this->get_parameter("lmin_Ang_OTS",lmin_Ang_OTS);

        minc_des << 1, -1, 1, -1,
		            1, -1, -1, 1;
        
        des_qind = 0.01*ts;

        mq_ind_mod = Eigen::Matrix<double,4,-1>::Zero(4,4);

        RCLCPP_INFO(this->get_logger(), "Creating communication");

        sub_ref.subscribe(this, "ref_pose");
        sub_x.subscribe(this, "x_coord");
        sub_ots.subscribe(this, "ang_ots");

        sync_.reset(new Synchronizer(SyncPolicy(1), sub_ref, sub_x, sub_ots));
        sync_->registerCallback(std::bind(&SingEvader::topic_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        publisher_ = this->create_publisher<pr_msgs::msg::PRArrayH>("ref_pose_mod", 1);


    }

    void SingEvader::topic_callback(const pr_msgs::msg::PRArrayH::ConstPtr& ref_msg,
                                    const pr_msgs::msg::PRArrayH::ConstPtr& x_msg,
                                    const pr_msgs::msg::PROTS::ConstPtr& ots_msg)
    {
        //Convert to Eigen
        for(int i=0;i<4;i++) {
            x_coord(i) = x_msg->data[i];
            q_ref(i) = ref_msg->data[i];
        }

        for(int i=0; i<ots_msg->ots.data.size(); i++) {
            int row = i/OTS.cols();
            int col = i%OTS.cols();
            OTS.coeffRef(row,col) = ots_msg->ots.data[i];
        }

        for(int i=0;i<6;i++)
            angOTS(i) = ots_msg->ots_ang[i];

        //Wait for beginning of experiment        
        bool enable = false;

        if (t_activation/ts <= iterations)
            enable = true;

        if (enable)
            std::cout << q_ind_mod << std::endl;

        q_ind_mod = PRSingularity::CalculateQindMod(
            x_coord,
            q_ref,
            angOTS,
            OTS,
            minc_des,
            robot_params,
            vc_des,
            mq_ind_mod,
            des_qind,
            lmin_Ang_OTS,
            tol,
            iter_max,
            tol_OTS,
            iter_OTS,
            ncomb,
            enable
        );

        iterations++;

    }


}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_ref_gen::SingEvader)