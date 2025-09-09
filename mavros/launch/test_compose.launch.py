from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Generate launch description for MAVROS composable node."""

    container = ComposableNodeContainer(
        name="mavros_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="mavros",
                plugin="mavros::router::Router",  # Or the specific MAVROS component you need
                name="mavros_router",
                parameters=[
                    {"fcu_urls": ["tcp://localhost:6777"]},
                    {"fcu_protocol": "v2.0"},
                    {"uas_urls": ["/uas1"]},
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        ],
        output="screen",
    )

    return LaunchDescription([container])
