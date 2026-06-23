import os
import xacro
from typing import Any, cast
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    def launch_setup(context, *_args, **_kwargs):
        pkg_path = get_package_share_directory("oh_my_hans_description")
        xacro_file = os.path.join(pkg_path, "description", "oh_my_hans_robot.urdf.xacro")
        robot_namespace = namespace.perform(context)
        robot_description_config = cast(
            Any,
            xacro.process_file(
                xacro_file,
                mappings={
                    "robot_namespace": robot_namespace,
                    "use_fake_hardware": use_fake_hardware.perform(context),
                },
            ),
        )
        robot_description_raw = robot_description_config.toxml()

        node_robot_state_publisher = Node(
            namespace=robot_namespace,
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[
                {"robot_description": robot_description_raw, "use_sim_time": True},
                {"frame_prefix": f"/{robot_namespace}/"},
            ],
            remappings=[("/joint_states", f"/{robot_namespace}/joint_states")],
        )

        return [node_robot_state_publisher]

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="oh_my_hans", description="Robot namespace"),
        DeclareLaunchArgument("use_fake_hardware", default_value="false", description="Use fake hardware interfaces"),
        OpaqueFunction(function=launch_setup),
    ])
