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
                    # {"fcu_urls": ["tcp://127.0.0.1:5760"]},
                    {"fcu_urls": ["udp://0.0.0.0:14540@"]},
                    {"gcs_urls": ["udp://127.0.0.1:14555@"]},
                    {"uas_urls": ["/uas1", "/uas2"]},
                    {"fcu_protocol": "v2.0"},
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="mavros",
                plugin="mavros::uas::UAS",  # Or the specific MAVROS component you need
                name="UAS1",
                parameters=[
                    {"uas_url": "/uas1"},
                    {"fcu_protocol": "v2.0"},
                    {"plugin_allowlist": ["sys_*", ]},
                    {"plugin_denylist": ["*"]},
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
         arguments=['--ros-args', '--log-level', 'DEBUG'],
    )

    return LaunchDescription([container])
