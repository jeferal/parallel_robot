import os
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

from numpy import fromstring

import yaml


def generate_launch_description():

    """Generate launch description with multiple components."""

    #Load config file

    robot_parameters_file = os.path.join(
        get_package_share_directory('pr_bringup'),
        'config',
        'pr_config_params.yaml'
    )

    controller_params_file = os.path.join(
        get_package_share_directory('pr_bringup'),
        'config',
        'pr_gus.yaml'
    )

    controller_yaml_file = open(controller_params_file)
    controller_params = yaml.load(controller_yaml_file)

    robot = controller_params['robot']['robot_name']
    robot_config = controller_params['robot']['config']

    robot_yaml_file = open(robot_parameters_file)
    pr_params = yaml.load(robot_yaml_file)    

    pr_config_params = pr_params[robot]['config'][robot_config]
    
    ref_file = controller_params['ref_path']
    
    with open(ref_file, 'r') as f:
        first_reference = fromstring(f.readline(), dtype=float, sep=" ").tolist()
    
    pr_gus = ComposableNodeContainer(
            node_name='pr_container',
            node_namespace='',
            package='rclcpp_components',
            node_executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='pr_sensors_actuators',
                    node_plugin='pr_sensors_actuators::MotorsSimulink',
                    node_name='motors',
                    remappings=[
                        ("control_action", "control_action"),
                        ("voltaje_sim", "voltaje_sim")
                    ],
                    parameters=[
                        {"vp_conversion": controller_params['actuators']['vp_conversion']},
                        {"max_v": controller_params['actuators']['v_sat']}
                    ]
                ),
                
                ComposableNode(
                    package='pr_controllers',
                    node_plugin='pr_controllers::GusController',
                    node_name='controller',
                    remappings=[
                        ("ref_pose", "ref_pose"),
                        ("joint_position", "joint_position"),
                        ("joint_velocity", "joint_velocity")
                    ],
                    parameters=[
                        {"k1": controller_params['controller']['k1']},
                        {"k2": controller_params['controller']['k1']},
                        {"ts": controller_params['ts']},
                        {"initial_position": first_reference},
                        {"initial_reference": first_reference}
                    ]
                ),
                ComposableNode(
                    package='pr_aux',
                    node_plugin='pr_aux::Derivator',
                    node_name='derivator',
                    remappings=[
                        ("joint_position", "joint_position"),
                        ("joint_velocity", "joint_velocity")
                    ],
                    parameters=[
                        {"initial_value": first_reference},
                        {"ts": controller_params['ts']}
                    ]
                ),
                ComposableNode(
                    package='pr_ref_gen',
                    node_plugin='pr_ref_gen::RefPose',
                    node_name='ref_pose_gen',
                    remappings=[
                        ("ref_pose", "ref_pose"),
                        ("end_flag", "end_flag"),
                        ("joint_position", "joint_position")
                    ],
                    parameters=[
                        {"ref_path": ref_file},
                        {"is_cart": False},
                        {"robot_config_params": pr_config_params}
                    ]
                ),
                ComposableNode(
                    package='pr_sensors_actuators',
                    node_plugin='pr_sensors_actuators::EncodersSimulink',
                    node_name='position_sensors',
                    remappings=[
                        ("joint_position", "joint_position")
                    ],
                    parameters=[
                        {"ts_ms": controller_params['ts']*1000},
                        {"initial_position": first_reference}
                    ]
                ),
            ],
            output='screen',
    )

    return launch.LaunchDescription([pr_gus])