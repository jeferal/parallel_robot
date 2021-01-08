#include <memory>


#include "pr_sensors_actuators/encoders_sim.hpp"
#include "pr_ref_gen/ref_pose.hpp"
#include "pr_aux/derivator.hpp"
#include "pr_controllers/gus_controller.hpp"

#include "rclcpp/rclcpp.hpp"


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;

    //Components
    auto encoders = std::make_shared<pr_sensors_actuators::EncodersSim>(options);
    exec.add_node(encoders);

    auto ref_gen = std::make_shared<pr_ref_gen::RefPose>(options);
    exec.add_node(ref_gen);

    auto derivator = std::make_shared<pr_aux::Derivator>(options);
    exec.add_node(derivator);

    auto controller = std::make_shared<pr_controllers::GusController>(options);
    exec.add_node(controller);

    exec.spin();

    rclcpp::shutdown();

    return 0;
}