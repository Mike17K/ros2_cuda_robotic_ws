import os
import yaml
import tempfile
from typing import Any, cast
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.actions import Node
import xacro
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import Node
from launch_param_builder import ParameterBuilder, load_yaml, load_xacro


def generate_launch_description():
    """
    Group A Bringup - Universal & Modular Configuration.
    Loads planning pipelines directly from local YAML configuration.
    """
    use_fake_hardware_arg = DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="true",
        description="Use mock_components/GenericSystem (true) or real hardware drivers (false)",
    )
    sim_gazebo_arg = DeclareLaunchArgument(
        "sim_gazebo",
        default_value="false",
        description="Switch to true if launching inside a Gazebo Simulation environment",
    )
    lift_type_arg = DeclareLaunchArgument("lift_type", default_value="ur_620", description="Ewellix model type")
    ur_type_arg = DeclareLaunchArgument("ur_type", default_value="ur10", description="UR robot type")
    parent_link_arg = DeclareLaunchArgument("parent_link", default_value="world", description="Parent link in the workcell")
    xyz_arg = DeclareLaunchArgument("xyz", default_value="0.0 0.0 0.0", description="Robot spawn position")
    rpy_arg = DeclareLaunchArgument("rpy", default_value="0.0 0.0 0.0", description="Robot spawn orientation")

    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Prefix for all TF frames (useful for multi-robot setups)",
    )

    def launch_setup(context):
        pkg_description = get_package_share_directory("group_a_description")
        pkg_moveit = get_package_share_directory("group_a_moveit_config")
        pkg_bringup = get_package_share_directory("group_a_bringup")

        # ── Runtime values ───────────────────────────────────────────────────────
        use_fake_hardware = LaunchConfiguration("use_fake_hardware").perform(context)
        sim_gazebo = LaunchConfiguration("sim_gazebo").perform(context)
        lift_type = LaunchConfiguration("lift_type").perform(context)
        ur_type = LaunchConfiguration("ur_type").perform(context)
        parent_link = LaunchConfiguration("parent_link").perform(context)
        xyz = LaunchConfiguration("xyz").perform(context)
        rpy = LaunchConfiguration("rpy").perform(context)

        current_namespace = context.launch_configurations.get("namespace", "")
        runtime_namespace = current_namespace if current_namespace else "group_a"
        runtime_namespace = runtime_namespace.strip("/")
        print(f"Current ROS Namespace: '{current_namespace}'")

        # ── Controllers YAML (namespace-substituted) ─────────────────────────────────
        controllers_template_path = os.path.join(pkg_bringup, "config", "controllers.yaml")
        with open(controllers_template_path, "r") as f:
            controllers_content = f.read()
        controllers_content = controllers_content.replace("$(var tf_prefix)", f"{runtime_namespace}/")

        controllers_tmp = tempfile.NamedTemporaryFile(
            mode="w",
            prefix=f"{runtime_namespace}_controllers_",
            suffix=".yaml",
            delete=False,
        )
        controllers_tmp.write(controllers_content)
        controllers_tmp.flush()
        controllers_tmp_path = controllers_tmp.name
        controllers_tmp.close()
        print(f"Generating controllers YAML for namespace '{runtime_namespace}' at '{controllers_tmp_path}'")

        # ── Xacro ────────────────────────────────────────────────────────────────
        xacro_file = os.path.join(pkg_description, "urdf", "group_a.urdf.xacro")
        robot_description_config = cast(
            Any,
            xacro.process_file(
                xacro_file,
                mappings={
                    "parent": parent_link,
                    "xyz": xyz,
                    "rpy": rpy,
                    "lift_type": lift_type,
                    "ur_type": ur_type,
                    "sim_gazebo": sim_gazebo,
                    "use_fake_hardware": use_fake_hardware,
                    "simulation_controllers": controllers_tmp_path,
                    "namespace": runtime_namespace,
                },
            ),
        )
        robot_desc = {"robot_description": robot_description_config.toxml()}

        # ── MoveIt SRDF ──────────────────────────────────────────────────────────
        srdf_file = os.path.join(pkg_description, "config", "combined_system.srdf.xacro")
        srdf_content = cast(
            Any,
            xacro.process_file(
                srdf_file,
                mappings={"tf_prefix": f"{runtime_namespace}/"},
            ),
        )
        robot_desc_semantic = {"robot_description_semantic": srdf_content.toxml()}

        # ── Kinematics (from autogen MoveIt package) ─────────────────────────────
        kinematics_file = os.path.join(pkg_moveit, "config", "kinematics.yaml")
        with open(kinematics_file, "r") as f:
            raw_kinematics = yaml.safe_load(f)
        kinematics_params = {"robot_description_kinematics": raw_kinematics}

        # ── Gz bridge YAML (same pattern as sensors) ─────────────────────────────
        template_bridge_yaml = os.path.join(pkg_bringup, "config", "gz_bridge.yaml")
        generated_bridge_yaml = f"/tmp/{runtime_namespace}_gz_bridge.yaml"
        with open(template_bridge_yaml, "r") as f:
            bridge_content = f.read()
        print(f"Generating bridge YAML for namespace '{runtime_namespace}' at '{generated_bridge_yaml}'")
        bridge_content = bridge_content.replace("{namespace}", runtime_namespace)
        with open(generated_bridge_yaml, "w") as f:
            f.write(bridge_content)

        # ── 1. Robot State Publisher ─────────────────────────────────────────────
        robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[
                robot_desc,
                {"use_sim_time": LaunchConfiguration("sim_gazebo")},
            ],
        )

        # ── 2. Standalone Controller Manager (real hardware only) ─────────────────
        controller_manager_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            parameters=[
                robot_desc, 
                ParameterFile(controllers_template_path, allow_substs=True),
                {"use_sim_time": LaunchConfiguration("sim_gazebo")},
            ],
            condition=UnlessCondition(LaunchConfiguration("sim_gazebo")),
            remappings=[("/robot_description", f"{runtime_namespace}/robot_description")],
        )

        # ── 3. Gazebo Spawner ────────────────────────────────────────────────────
        gazebo_spawn_robot = Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=[
                "-topic",
                "robot_description",
                "-name",
                runtime_namespace,
            ],
            condition=IfCondition(LaunchConfiguration("sim_gazebo")),
        )

        # ── 4. Camera Bridge ─────────────────────────────────────────────────────
        camera_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="camera_bridge",
            output="screen",
            parameters=[{"use_sim_time": True}],
            arguments=["--ros-args", "-p", f"config_file:={generated_bridge_yaml}"],
            condition=IfCondition(LaunchConfiguration("sim_gazebo")),
        )

        # ── 5. Controller Spawners ───────────────────────────────────────────────
        motion_default_active_controllers_spawner = Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[
                "joint_state_broadcaster",
                "lift_joint_trajectory_controller",
                "ur_joint_trajectory_controller",
                "--controller-manager",
                f"/{runtime_namespace}/controller_manager",
            ],
        )

        # ── 6. MoveIt move_group ─────────────────────────────────────────────────
        # Parameter loading order matters: last entry wins on key conflicts.
        #   - planning_params (dict, scalars only) → overrides any autogen pipeline keys
        #   - sensors_tmp_path (file path string)  → ROS 2 reads list params from file
        #   - octomap scalars dict                 → simple key/value, safe as dict
        moveit_config = (
            MoveItConfigsBuilder(runtime_namespace, package_name="group_a_moveit_config")
            .planning_scene_monitor(
                publish_geometry_updates=True,
                publish_state_updates=True,
                publish_transforms_updates=True,
                publish_planning_scene=True,
                publish_robot_description=True,
                publish_robot_description_semantic=True,
            )
            .to_moveit_configs()
        )
        move_group_node = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                robot_desc,
                robot_desc_semantic,
                kinematics_params,
                ParameterFile(os.path.join(pkg_moveit, "config", "joint_limits.yaml"), allow_substs=True),
                ParameterFile(os.path.join(pkg_moveit, "config", "planning.yaml"), allow_substs=True),
                {"use_sim_time": LaunchConfiguration("sim_gazebo")},
                {
                    "octomap_frame": "world",
                    "octomap_resolution": 0.05,
                    "max_range": 3.0,
                    "workspace_bounds": {
                        "min_x": -5.0,
                        "min_y": -5.0,
                        "min_z": -2.0,
                        "max_x": 5.0,
                        "max_y": 5.0,
                        "max_z": 5.0,
                    },
                },
                ParameterFile(os.path.join(pkg_bringup, "config", "sensors_3d.yaml"), allow_substs=True),
            ],
            remappings=[
                ("/robot_description", f"{runtime_namespace}/robot_description"),
                ("/robot_description_semantic", f"{runtime_namespace}/robot_description_semantic"),
            ],
        )

        return [
            robot_state_publisher,
            controller_manager_node,
            gazebo_spawn_robot,
            camera_bridge,
            move_group_node,
            TimerAction(
                period=4.0,
                actions=[motion_default_active_controllers_spawner],
            ),
        ]

    return LaunchDescription(
        [
            use_fake_hardware_arg,
            sim_gazebo_arg,
            lift_type_arg,
            ur_type_arg,
            parent_link_arg,
            namespace_arg,
            xyz_arg,
            rpy_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
