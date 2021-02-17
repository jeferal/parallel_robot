import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():

    first_reference_q = [0.695245, 0.691370, 0.693736, 0.654154]

    pr_rl = ComposableNodeContainer(
                node_name='pr_container',
                node_namespace='',
                package='rclcpp_components',
                node_executable='component_container',
                composable_node_descriptions=[
                    ComposableNode(
                    package='pr_aux',
                    node_plugin='pr_aux::Derivator',
                    node_name='derivator',
                    remappings=[
                        ("joint_position", "joint_position"),
                        ("joint_velocity", "joint_velocity")
                    ],
                    parameters=[
                        {"initial_value": first_reference_q},
                        {"ts": 0.01}
                    ]
                ),
                ComposableNode(
                    package='pr_sensors_actuators',
                    node_plugin='pr_sensors_actuators::EncodersSimulink',
                    node_name='position_sensor',
                    remappings=[
                        ("joint_position", "joint_position"),
                        ("posicion_sim", "posicion_sim"),
                    ],
                    parameters=[
                        {"ts_ms": 10.0},
                        {"initial_position": first_reference_q},
                    ]
                ),
                ComposableNode(
                    package='pr_sensors_actuators',
                    node_plugin='pr_sensors_actuators::MotorsSimulink',
                    node_name='actuators',
                    remappings=[
                        ("control_action", "control_action"),
                        ("voltaje_sim", "voltaje_sim"),
                    ],
                    parameters=[
                        {"vp_conversion": [1.0, 1.0, 1.0, 1.0]},
                        {"max_v": 9.5},
                    ]
                ),
                ComposableNode(
                    package='pr_modelling',
                    node_plugin='pr_modelling::StatePublisher',
                    node_name='state_publisher',
                    remappings=[
                        ("joint_position", "joint_position"),
                        ("joint_velocity", "joint_velocity"),
                        ("pr_state", "pr_state"),
                    ],
                ),
            ],
            output='screen',
    )

    return launch.LaunchDescription([pr_rl])