import os
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

from numpy import fromstring, pi

def LoadConfiguration(config=1):

    param_list = [
        #R1    R2    R3    ds     betaFD     betaFI    Rm1   Rm2   Rm3    betaMD     betaMI
        #config 0
        [0.4 , 0.4 , 0.4 , 0.0 , 50*pi/180, 40*pi/180, 0.2 , 0.2 , 0.2 , 30*pi/180, 40*pi/180],
        #config 1
        [0.4 , 0.4 , 0.4 , 0.15, 90*pi/180, 45*pi/180, 0.3 , 0.3 , 0.3 , 50*pi/180, 90*pi/180],
        #config 2
        [0.4 , 0.4 , 0.4 , 0.15, 90*pi/180, 45*pi/180, 0.25, 0.25, 0.25, 5*pi/180 , 90*pi/180],
        #config 3
        [0.3 , 0.3 , 0.3 , 0.0 , 5*pi/180 , 90*pi/180, 0.2 , 0.2 , 0.2 , 70*pi/180, 30*pi/180],
        #config 4
        [0.4 , 0.4 , 0.4 , 0.15, 90*pi/180, 5*pi/180 , 0.15, 0.15, 0.15, 45*pi/180, 90*pi/180],
        #config 5
        [0.3 , 0.3 , 0.3 , 0.15, 50*pi/180, 90*pi/180, 0.15, 0.15, 0.15, 90*pi/180, 45*pi/180],
        #config 6
        [0.25, 0.25, 0.25, 0.15, 5*pi/180 , 90*pi/180, 0.15, 0.15, 0.15, 80*pi/180, 75*pi/180],
        #config 7
        [0.35, 0.35, 0.35, 0.15, 90*pi/180, 25*pi/180, 0.2 , 0.2 , 0.2 , 60*pi/180, 90*pi/180],
        #congig 8
        [0.35, 0.35, 0.35, 0.15, 35*pi/180, 30*pi/180, 0.15, 0.15, 0.15, 5*pi/180 , 90*pi/180]
    ]

    return param_list[config]

def generate_launch_description():

    """Generate launch description with multiple components."""

    #Load config file
    """
    config = os.path.join(
        get_package_share_directory('pr_bringup'),
        'config',
        'pr_gus.yaml'
    )
    """

    ref_file = "/home/paralelo4dofnew/parallel_robot_ws/references/refeprism_pata1234.txt"
    
    with open(ref_file, 'r') as f:
        first_reference = fromstring(f.readline(), dtype=float, sep=" ").tolist()

    robot_config_params = LoadConfiguration(config=1)

    print(robot_config_params)

    pr_gus = ComposableNodeContainer(
            node_name='pr_container',
            node_namespace='',
            package='rclcpp_components',
            node_executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='pr_sensors_actuators',
                    node_plugin='pr_sensors_actuators::Motor',
                    node_name='motor_0',
                    remappings=[
                        ("control_action", "control_action"),
                        ("end_flag", "end_flag")
                    ],
                    parameters=[
                        {"vp_conversion": [1.0, 1.0, 1.0, 1.0]},
                        {"max_v": 3.0}
                    ]
                ),
                ComposableNode(
                    package='pr_sensors_actuators',
                    node_plugin='pr_sensors_actuators::Motor',
                    node_name='motor_1',
                    remappings=[
                        ("control_action", "control_action"),
                        ("end_flag", "end_flag")
                    ],
                    parameters=[
                        {"vp_conversion": [1.0, 1.0, 1.0, 1.0]},
                        {"max_v": 3.0}
                    ]
                ),
                ComposableNode(
                    package='pr_sensors_actuators',
                    node_plugin='pr_sensors_actuators::Motor',
                    node_name='motor_2',
                    remappings=[
                        ("control_action", "control_action"),
                        ("end_flag", "end_flag")
                    ],
                    parameters=[
                        {"vp_conversion": [1.0, 1.0, 1.0, 1.0]},
                        {"max_v": 3.0}
                    ]
                ),
                ComposableNode(
                    package='pr_sensors_actuators',
                    node_plugin='pr_sensors_actuators::Motor',
                    node_name='motor_3',
                    remappings=[
                        ("control_action", "control_action"),
                        ("end_flag", "end_flag")
                    ],
                    parameters=[
                        {"vp_conversion": [1.0, 1.0, 1.0, 1.0]},
                        {"max_v": 3.0}
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
                        {"k1": 0.75},
                        {"k2": 0.5},
                        {"ts": 0.01},
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
                        {"ts": 0.01}
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
                        {"robot_config_params": robot_config_params}
                    ]
                ),
                ComposableNode(
                    package='pr_sensors_actuators',
                    node_plugin='pr_sensors_actuators::Encoders',
                    node_name='position_sensors',
                    remappings=[
                        ("joint_position", "joint_position")
                    ],
                    parameters=[
                        {"ts_ms": 10.0},
                        {"initial_position": first_reference}
                    ]
                ),
            ],
            output='screen',
    )

    return launch.LaunchDescription([pr_gus])