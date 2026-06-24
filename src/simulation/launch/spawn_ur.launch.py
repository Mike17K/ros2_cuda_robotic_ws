import os
from typing import Any, cast
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    """
    Spawn a standalone UR robot for visualization purposes.

    NOTE: ur_description's ur.urdf.xacro has no ros2_control block and no
    Gazebo plugin, so this spawner provides RSP + visual mesh only — joints
    cannot be commanded.  For a fully controlled UR use group_a_bringup which
    wraps the UR with a ros2_control hardware interface.
    """
    ns_arg = DeclareLaunchArgument(
        "namespace", default_value="ur10", description="Robot namespace"
    )
    ur_type_arg = DeclareLaunchArgument(
        "ur_type", default_value="ur10", description="UR type (ur3, ur5, ur10, ur10e …)"
    )

    x_pose = LaunchConfiguration("x", default="2.0")
    y_pose = LaunchConfiguration("y", default="2.0")
    z_pose = LaunchConfiguration("z", default="0.0")

    def launch_setup(context):
        pkg_description = get_package_share_directory("ur_description")
        robot_namespace = LaunchConfiguration("namespace").perform(context)
        ur_type = LaunchConfiguration("ur_type").perform(context)

        xacro_file = os.path.join(pkg_description, "urdf", "ur.urdf.xacro")

        robot_description_config = cast(
            Any,
            xacro.process_file(
                xacro_file,
                mappings={
                    "name": robot_namespace,
                    "ur_type": ur_type,
                    "tf_prefix": f"{robot_namespace}_",
                },
            ),
        )

        robot_desc = {"robot_description": robot_description_config.toxml()}

        robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            namespace=robot_namespace,
            parameters=[
                robot_desc,
                {"use_sim_time": True},
            ],
        )

        spawn_ur = Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=[
                "-string", robot_description_config.toxml(),
                "-name", robot_namespace,
                "-allow_renaming", "true",
                "-x", x_pose,
                "-y", y_pose,
                "-z", z_pose,
            ],
        )

        return [robot_state_publisher, spawn_ur]

    return LaunchDescription([
        ns_arg,
        ur_type_arg,
        OpaqueFunction(function=launch_setup),
    ])
