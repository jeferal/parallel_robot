#ifndef PR_MOCAP__PR_X_MOCAP_HPP_
#define PR_MOCAP__PR_X_MOCAP_HPP_

#include "rclcpp/rclcpp.hpp"

#include "pr_msgs/msg/pr_array_h.hpp"
#include "pr_msgs/msg/pr_mocap.hpp"

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include <NatNetTypes.h>
#include <NatNetCAPI.h>
#include <NatNetClient.h>

void NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, void* pUserData);    // receives data from the server

namespace pr_mocap
{
    class PRXMocap : public rclcpp::Node
    {
        public:
            explicit PRXMocap(const rclcpp::NodeOptions & options);

            ~PRXMocap();

            NatNetClient* g_pClient = NULL;
            sServerDescription g_serverDescription;
            Eigen::Matrix<double, 3, 2> XCoords;
            pr_msgs::msg::PRMocap mocap_msg;

        protected:
            void topic_callback(const pr_msgs::msg::PRArrayH::SharedPtr q_msg);
            int ConnectClient();
            int error_calc(const double &tol, double &error, const std::array<double, 4> &x_mocap, const std::array<double, 4> &x_model);

        private:
            sNatNetClientConnectParams g_connectParams;
            int g_analogSamplesPerMocapFrame = 0;

            rclcpp::Publisher<pr_msgs::msg::PRMocap>::SharedPtr publisher_;
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr subscription_;
    };

}

#endif // PR_MOCAP__PR_X_MOCAP_HPP_