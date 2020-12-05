import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from numpy import fromstring

def generate_launch_description():

    """Generate launch description with multiple components."""
    with open('/home/paralelo4dofnew/parallel_robot_ws/references/ref_qinde_TRR0_CF1_IdV1.txt', 'r') as f:
        first_reference = fromstring(f.readline(), dtype=float, sep=" ").tolist()
    
    encoders = ComposableNodeContainer(
            node_name='pr_container',
            node_namespace='',
            package='rclcpp_components',
            node_executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='pr_sensors_actuators',
                    node_plugin='pr_sensors_actuators::Encoders',
                    node_name='position_sensors',
                    remappings=[
                        ("joint_position", "joint_position")
                    ],
                    parameters=[
                        {"ts": 10.0},
                        {"cond_inicial": first_reference}
                    ]
                )
            ],
            output='screen',
    )

    return launch.LaunchDescription([encoders])