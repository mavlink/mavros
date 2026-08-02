import os

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def _build_container(context: LaunchContext, fcu_url, gcs_url):
    """Build the composable container once launch arguments are resolved."""
    fcu = context.perform_substitution(fcu_url)
    gcs = context.perform_substitution(gcs_url)

    # Pick a container executable: the Callback Group Events executor is only
    # available on Lyrical+ (rclcpp >= 30.0.0).
    distro = os.environ.get("ROS_DISTRO", "")
    if distro in ("lyrical", "rolling"):
        container_executable = "component_container"
        container_arguments = ["--executor-type", "events-cbg"]
    else:
        container_executable = "component_container_mt"
        container_arguments = []

    container = ComposableNodeContainer(
        name="mavros_container",
        namespace="",
        package="rclcpp_components",
        executable=container_executable,
        composable_node_descriptions=[
            ComposableNode(
                package="mavros",
                plugin="mavros::router::Router",
                name="mavros_router",
                parameters=[
                    {"fcu_urls": [fcu]},
                    {"gcs_urls": [gcs]},
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
        arguments=container_arguments + ["--ros-args", "--log-level", "DEBUG"],
        output="screen",
    )

    return [container]


def generate_launch_description():
    """Generate launch description for MAVROS composable node."""
    fcu_url = LaunchConfiguration("fcu_url")
    gcs_url = LaunchConfiguration("gcs_url")

    return LaunchDescription([
        DeclareLaunchArgument(
            "fcu_url", default_value="udp://0.0.0.0:14540@",
            description="FCU connection URL"
        ),
        DeclareLaunchArgument(
            "gcs_url", default_value="udp://127.0.0.1:14555@",
            description="GCS connection URL"
        ),
        OpaqueFunction(function=_build_container, args=[fcu_url, gcs_url]),
    ])
