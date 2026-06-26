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
    Group A ROS2-control bringup — no Gazebo.

    Launches under /group_a namespace:
      • robot_state_publisher
      • ros2_control_node  (controller manager)
      • joint_state_broadcaster
      • lift_joint_trajectory_controller
      • ur_joint_trajectory_controller

    For Gazebo simulation use spawn_gz.launch.py instead — the gz_ros2_control
    plugin inside Gazebo acts as the controller manager there.
    """
    use_fake_hardware_arg = DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="true",
        description="Use mock_components/GenericSystem (true) or real hardware drivers (false)",
    )
    lift_type_arg = DeclareLaunchArgument("lift_type", default_value="ur_620", description="Ewellix model type")
    ur_type_arg = DeclareLaunchArgument("ur_type", default_value="ur10", description="UR robot type")

    def launch_setup(context):
        pkg_description = get_package_share_directory("group_a_description")
        pkg_control = get_package_share_directory("group_a_control")

        namespace = "group_a"
        use_fake_hardware = LaunchConfiguration("use_fake_hardware").perform(context)
        lift_type = LaunchConfiguration("lift_type").perform(context)
        ur_type = LaunchConfiguration("ur_type").perform(context)

        xacro_file = os.path.join(pkg_description, "urdf", "group_a.urdf.xacro")
        controllers_yaml = os.path.join(pkg_control, "config", "group_a_controllers.yaml")

        robot_description_config = cast(
            Any,
            xacro.process_file(
                xacro_file,
                mappings={
                    "lift_type": lift_type,
                    "ur_type": ur_type,
                    "sim_gazebo": "false",
                    "use_fake_hardware": use_fake_hardware,
                },
            ),
        )

        robot_desc = {"robot_description": robot_description_config.toxml()}

        robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            namespace=namespace,
            parameters=[robot_desc],
        )

        # Standalone controller manager (not from Gazebo plugin)
        controller_manager_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            namespace=namespace,
            parameters=[robot_desc, controllers_yaml],
        )

        controller_manager = f"/{namespace}/controller_manager"

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

        return [
            robot_state_publisher,
            controller_manager_node,
            TimerAction(period=2.0, actions=[motion_default_active_controllers_spawner, motion_default_inactive_controllers_spawner]),
        ]

    return LaunchDescription(
        [
            use_fake_hardware_arg,
            lift_type_arg,
            ur_type_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
