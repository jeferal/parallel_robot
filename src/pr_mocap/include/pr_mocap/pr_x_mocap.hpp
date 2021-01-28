#ifndef PR_MOCAP__PR_X_MOCAP_HPP_
#define PR_MOCAP__PR_X_MOCAP_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

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
            Eigen::Matrix<double, 3, 6> MarkersMatrix;
            pr_msgs::msg::PRMocap mocap_msg;
            std::vector<double> markers_ids;
            rclcpp::Publisher<pr_msgs::msg::PRMocap>::SharedPtr publisher_;
            bool robot_5p;
            std::vector<std::string> markers_name{"P_Movil1_1", "P_Movil1_2", "P_Movil1_3", "P_Fija1_1", "P_Fija1_2", "P_Fija1_3"};
            int markers_pos[6] = {0, 1, 2, 3, 4, 5};

        protected:
            int ConnectClient();

        private:
            sNatNetClientConnectParams g_connectParams;
            int g_analogSamplesPerMocapFrame = 0;
            double tol;
            std::string server_address;
            int server_command_port, server_data_port;
    };

}

#endif // PR_MOCAP__PR_X_MOCAP_HPP_