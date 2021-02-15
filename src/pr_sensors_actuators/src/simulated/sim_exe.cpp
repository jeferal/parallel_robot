#include "rclcpp/rclcpp.hpp"
#include "pr_sensors_actuators/simulated/simulink_socket_interface.hpp"


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    //Multithread executor
    rclcpp::executors::MultiThreadedExecutor executor;
    rclcpp::NodeOptions options;

    auto nh = std::make_shared<pr_sensors_actuators::SimulinkSocketInterface>(options);

    executor.add_node(nh);

    executor.spin();

    rclcpp::shutdown();

    return 0;
}