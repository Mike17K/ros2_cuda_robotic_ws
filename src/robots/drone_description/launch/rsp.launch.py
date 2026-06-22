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

    def launch_setup(context, *args, **kwargs):
        pkg_path = os.path.join(get_package_share_directory("drone_description"))
        xacro_file = os.path.join(pkg_path, "description", "drone.urdf.xacro")
        robot_namespace = namespace.perform(context)
        robot_description_config = cast(
            Any,
            xacro.process_file(
                xacro_file,
                mappings={"robot_namespace": robot_namespace},
            ),
        )
        robot_description_raw = robot_description_config.toxml()

        node_robot_state_publisher = Node(
            namespace=robot_namespace,
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description_raw, "use_sim_time": True}],
        )

        node_world_to_map_tf = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            arguments=["0", "0", "0", "0", "0", "0", "world", "map"],
        )

        return [node_robot_state_publisher, node_world_to_map_tf]

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="drone", description="Robot namespace"),
        OpaqueFunction(function=launch_setup),
    ])
