import os
from typing import Any, cast
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # 1. Δήλωση των Launch Arguments για το Launch Script
    ns_arg = DeclareLaunchArgument("namespace", default_value="ewellix", description="Robot namespace")
    type_arg = DeclareLaunchArgument("type", default_value="tlt_x25", description="Ewellix lift model type")
    tf_prefix_arg = DeclareLaunchArgument("tf_prefix", default_value="lift_", description="TF prefix for the robot links")

    # Θέση αρχικής τοποθέτησης στο Gazebo
    x_pose = LaunchConfiguration("x", default="2.0")
    y_pose = LaunchConfiguration("y", default="2.0")
    z_pose = LaunchConfiguration("z", default="0.0")

    def launch_setup(context):
        pkg_description = get_package_share_directory("ewellix_description")

        # Λήψη των τιμών κατά το runtime
        robot_namespace = LaunchConfiguration("namespace").perform(context)
        lift_type = LaunchConfiguration("type").perform(context)
        tf_prefix = LaunchConfiguration("tf_prefix").perform(context)

        # Εντοπισμός του νέου Xacro αρχείου
        xacro_file = os.path.join(pkg_description, "urdf", "ewellix_lift.urdf.xacro")

        # Διαδρομές για τα αρχεία ρυθμίσεων (YAML)
        parameters_file = os.path.join(pkg_description, "config", f"{lift_type}.yaml")
        gazebo_controllers = os.path.join(pkg_description, "config", "control", "jtc.yaml")

        # 2. Parsing του Xacro με τα σωστά Mappings
        robot_description_config = cast(
            Any,
            xacro.process_file(
                xacro_file,
                mappings={
                    "type": lift_type,
                    "tf_prefix": tf_prefix,
                    "sim_gazebo": "true",
                    "use_fake_hardware": "false",
                    "generate_ros2_control_tag": "true",
                    "parameters_file": parameters_file,
                    "gazebo_controllers": gazebo_controllers,
                    "robot_namespace": robot_namespace,
                },
            ),
        )

        robot_desc = {"robot_description": robot_description_config.toxml()}

        # 3. Κόμβος Robot State Publisher
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

        # 4. Κόμβος Spawn στο Gazebo Sim (ros_gz_sim)
        # Entity name = robot_namespace so gz_ros2_control creates
        # controller_manager at /{robot_namespace}/controller_manager,
        # aligning with the RSP namespace and joint_states topic.
        spawn_ewellix_lift = Node(
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

        joint_state_broadcaster_spawner = Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=["joint_state_broadcaster", "--controller-manager", controller_manager],
        )

        lift_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=["lift_joint_trajectory_controller", "--controller-manager", controller_manager],
        )

        return [
            robot_state_publisher,
            spawn_ewellix_lift,
            # Delay spawners until Gazebo has loaded the plugin and started the controller manager
            TimerAction(
                period=5.0,
                actions=[joint_state_broadcaster_spawner, lift_controller_spawner],
            ),
        ]

    return LaunchDescription(
        [
            ns_arg,
            type_arg,
            tf_prefix_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
