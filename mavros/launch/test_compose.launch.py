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
                plugin="mavros::router::Router",
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
                plugin="mavros::uas::UAS",
                name="UAS1",
                namespace="drone1",
                parameters=[
                    {"uas_url": "/uas1"},
                    {"fcu_protocol": "v2.0"},
                    # {"plugin_allowlist": ["sys_*"]},
                    # {"plugin_denylist": ["*"]},
                    {"system_id": 1},
                    {"target_system_id": 1},
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="mavros",
                plugin="mavros::uas::UAS",
                name="UAS2",
                namespace="drone2",
                parameters=[
                    {"uas_url": "/uas2"},
                    {"fcu_protocol": "v2.0"},
                    {"plugin_allowlist": ["sys_*"]},
                    {"plugin_denylist": ["*"]},
                    {"system_id": 2},
                    {"target_system_id": 2},
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
        arguments=["--ros-args", "--log-level", "DEBUG"],
    )

    return LaunchDescription([container])
