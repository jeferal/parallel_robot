import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

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

    ref_file_q = "/home/paralelo4dofnew/parallel_robot_ws/references/refeprism_indep_TRR8.txt"
    ref_file_x = "/home/paralelo4dofnew/parallel_robot_ws/references/refecart_TRR8_identificar.txt"

    robot_config_params = LoadConfiguration(config=1)

    with open(ref_file_q, 'r') as f:
        first_reference_q = fromstring(f.readline(), dtype=float, sep=" ").tolist()
    
    with open(ref_file_x, 'r') as f:
        first_reference_x = fromstring(f.readline(), dtype=float, sep=" ").tolist()

    pr_pdg = ComposableNodeContainer(
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
                        {"vp_conversion": [28.4628, 28.4628, 28.4628, 246.6779]},
                        {"max_v": 5.0}
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
                        {"vp_conversion": [28.4628, 28.4628, 28.4628, 246.6779]},
                        {"max_v": 5.0}
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
                        {"vp_conversion": [28.4628, 28.4628, 28.4628, 246.6779]},
                        {"max_v": 5.0}
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
                        {"vp_conversion": [28.4628, 28.4628, 28.4628, 246.6779]},
                        {"max_v": 5.0}
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
                        {"initial_value": first_reference_q},
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
                        {"ref_path": ref_file_q},
                        {"is_cart": False},
                        {"robot_config_params": robot_config_params}
                    ]
                ),

                ComposableNode(
                    package='pr_modelling',
                    node_plugin='pr_modelling::ForwardKinematics',
                    node_name='for_kin',
                    remappings=[
                        ("joint_position", "joint_position"),
                        ("x_coord", "x_coord"),
                    ],
                    parameters=[
                        {"robot_config_params": robot_config_params},
                        {"initial_position": first_reference_x},
                        {"tol": 0.0000001},
                        {"iter": 30},
                    ]
                ),

                ComposableNode(
                    package='pr_modelling',
                    node_plugin='pr_modelling::InverseKinematics',
                    node_name='inv_kin',
                    remappings=[
                        ("x_coord", "x_coord"),
                        ("q_sol", "q_sol"),
                    ],
                    parameters=[
                        {"robot_config_params": robot_config_params},
                    ]
                ),

                ComposableNode(
                    package='pr_modelling',
                    node_plugin='pr_modelling::IndependentJacobian',
                    node_name='ind_jac',
                    remappings=[
                        ("q_sol", "q_sol"),
                        ("ind_jac", "ind_jac"),
                    ],
                    parameters=[
                    ]
                ),

                ComposableNode(
                    package='pr_modelling',
                    node_plugin='pr_modelling::DependentJacobian',
                    node_name='dep_jac',
                    remappings=[
                        ("x_coord", "x_coord"),
                        ("q_sol", "q_sol"),
                        ("dep_jac", "dep_jac")
                    ],
                    parameters=[
                        {"robot_config_params": robot_config_params}
                    ]
                ),

                ComposableNode(
                    package='pr_modelling',
                    node_plugin='pr_modelling::RastT',
                    node_name='rast_t',
                    remappings=[
                        ("dep_jac", "dep_jac"),
                        ("ind_jac", "ind_jac"),
                        ("rast_t", "rast_t")
                    ],
                    parameters=[
                    ]
                ),

                ComposableNode(
                    package='pr_modelling',
                    node_plugin='pr_modelling::QGrav',
                    node_name='q_grav',
                    remappings=[
                        ("x_coord", "x_coord"),
                        ("q_sol", "q_sol"),
                        ("rast_t", "rast_t")
                    ]
                ),

                ComposableNode(
                    package='pr_controllers',
                    node_plugin='pr_controllers::PDGController',
                    node_name='controller',
                    remappings=[
                        ("ref_pose", "ref_pose"),
                        ("joint_position", "joint_position"),
                        ("joint_velocity", "joint_velocity"),
                        ("q_grav", "q_grav")
                    ],
                    parameters=[
                        {"kp_gain": [70000.0, 70000.0, 70000.0, 683716.0]},
                        {"kv_gain": [165.0, 165.0, 165.0, 12912.0]},
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
                        {"initial_position": first_reference_q}
                    ]
                ),
            ],
            output='screen',
    )

    return launch.LaunchDescription([pr_pdg])