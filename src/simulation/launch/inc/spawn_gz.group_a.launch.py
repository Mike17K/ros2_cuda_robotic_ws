import os
from typing import Any, cast
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    """
    Spawn Group A (Ewellix + UR + Orbbec) into a running Gazebo instance.

    This file is meant to be included by simulation/launch/gazebo.launch.py
    after Gazebo is already up.  It handles:
      • robot_state_publisher  (in /group_a namespace)
      • Gazebo entity spawn
      • Controller spawners    (joint_state_broadcaster, lift JTC, ur JTC)
      • ros_gz_bridge          (camera topics + pose)
    """
    ns_arg = DeclareLaunchArgument("namespace", default_value="group_a", description="Robot namespace")
    lift_type_arg = DeclareLaunchArgument("lift_type", default_value="ur_620", description="Ewellix model type")
    ur_type_arg = DeclareLaunchArgument("ur_type", default_value="ur10", description="UR robot type")

    x_pose = LaunchConfiguration("x", default="0.0")
    y_pose = LaunchConfiguration("y", default="0.0")
    z_pose = LaunchConfiguration("z", default="0.0")

    def launch_setup(context):
        pkg_description = get_package_share_directory("group_a_description")
        pkg_control = get_package_share_directory("group_a_control")
        pkg_bringup = get_package_share_directory("group_a_bringup")
        pkg_simulation = get_package_share_directory("simulation")

        robot_namespace = LaunchConfiguration("namespace").perform(context)
        lift_type = LaunchConfiguration("lift_type").perform(context)
        ur_type = LaunchConfiguration("ur_type").perform(context)

        xacro_file = os.path.join(pkg_description, "urdf", "group_a.urdf.xacro")
        simulation_controllers = os.path.join(pkg_control, "config", "group_a_controllers.yaml")
        bridge_params = os.path.join(pkg_simulation, "config", "gz_bridge.yaml")

        robot_description_config = cast(
            Any,
            xacro.process_file(
                xacro_file,
                mappings={
                    "namespace": robot_namespace,
                    "lift_type": lift_type,
                    "ur_type": ur_type,
                    "simulation_controllers": simulation_controllers,
                },
            ),
        )

        robot_desc = {"robot_description": robot_description_config.toxml()}

        # RSP runs in the robot namespace.  Since gz_ros2_control derives the
        # controller manager namespace from the Gazebo entity name, the entity
        # must be spawned with the same name as this namespace.
        robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            namespace=robot_namespace,
            parameters=[
                robot_desc,
                {"use_sim_time": True},
                {"frame_prefix": robot_namespace + "/"}
            ],
        )

        # Gazebo entity name = robot_namespace → controller manager at
        # /{robot_namespace}/controller_manager
        spawn_entity = Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=[
                "-string",
                robot_description_config.toxml(),
                "-name",
                robot_namespace,
                "-allow_renaming",
                "true",
                "-x",
                x_pose,
                "-y",
                y_pose,
                "-z",
                z_pose,
            ],
        )

        controller_manager = f"/{robot_namespace}/controller_manager"

        # 1. Broadcaster spawner: Starts automatically (ACTIVE) so TFs are published immediately
        joint_state_broadcaster_spawner = Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager",
                controller_manager,
            ],
        )

        # 2. Motion controllers spawner: Loaded into memory but kept INACTIVE
        motion_default_active_controllers_spawner = Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[
                "lift_joint_trajectory_controller",
                "ur_joint_trajectory_controller",
                # "all_joint_trajectory_controller",
                "--controller-manager",
                controller_manager,
            ],
        )
        motion_default_inactive_controllers_spawner = Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[
                # "lift_joint_trajectory_controller",
                # "ur_joint_trajectory_controller",
                "all_joint_trajectory_controller",
                "--inactive",
                "--controller-manager",
                controller_manager,
            ],
        )

        # Bridge Gazebo camera / pose topics into ROS 2.
        ros_gz_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="group_a_bridge",
            output="screen",
            parameters=[{"use_sim_time": True}],
            arguments=["--ros-args", "-p", f"config_file:={bridge_params}"],
        )

        return [
            robot_state_publisher,
            spawn_entity,
            ros_gz_bridge,
            # Delay both spawners by 5 seconds so Gazebo has time to initialize
            TimerAction(
                period=5.0, 
                actions=[joint_state_broadcaster_spawner, motion_default_active_controllers_spawner, motion_default_inactive_controllers_spawner]
            ),
        ]
    
    return LaunchDescription(
        [
            ns_arg,
            lift_type_arg,
            ur_type_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
