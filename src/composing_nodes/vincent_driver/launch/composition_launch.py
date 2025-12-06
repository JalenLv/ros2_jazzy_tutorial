from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name="a_buncha_nodes",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="vincent_driver",
                    plugin="palomino::VincentDriver",
                    name="vincent_driver",
                    extra_arguments=[{
                        "use_intra_process_comms": True,
                    }],
                ),
            ],
        ),
    ])
