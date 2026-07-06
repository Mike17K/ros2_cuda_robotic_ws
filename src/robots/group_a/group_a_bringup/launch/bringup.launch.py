import os
from typing import Any, cast
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import Node
from launch_param_builder import ParameterBuilder

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def get_launch_arguments() -> list[DeclareLaunchArgument]:
    args = []
    args.append(DeclareLaunchArgument("use_fake_hardware", default_value="true", description="Use mock_components/GenericSystem (true) or real hardware drivers (false)"))
    args.append(DeclareLaunchArgument("sim_gazebo", default_value="false", description="Switch to true if launching inside a Gazebo Simulation environment"))
    args.append(DeclareLaunchArgument("lift_type", default_value="ur_620", description="Ewellix model type"))
    args.append(DeclareLaunchArgument("ur_type", default_value="ur10", description="UR robot type"))
    args.append(DeclareLaunchArgument("parent_link", default_value="world", description="Parent link in the workcell"))
    args.append(DeclareLaunchArgument("xyz", default_value="0.0 0.0 0.0", description="Robot spawn position"))
    args.append(DeclareLaunchArgument("rpy", default_value="0.0 0.0 0.0", description="Robot spawn orientation"))
    args.append(DeclareLaunchArgument("namespace", default_value="", description="Namespace for the robot tf frames ,topics and nodes"))
    args.append(DeclareLaunchArgument("tf_prefix", default_value="", description="Prefix for all TF frames after namespace is applied"))
    return args


_param_file_refs: list[Any] = []


def _make_param_file(path, context):
    pf = ParameterFile(path, allow_substs=True)
    _param_file_refs.append(pf)  # prevent garbage collection / temp-file deletion
    return pf.evaluate(context)


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
    namespace = LaunchConfiguration("namespace").perform(context)

    # ── Controllers YAML (namespace-substituted) ─────────────────────────────────
    # DUBUGGING TIP! we need to keep the parameter file in an instance! it creates the tmp file when we call evaluate() on it
    # if we don't keep the instance, it will be garbage collected and the tmp file will be deleted before the node can read it
    joint_limits_file_path = _make_param_file(os.path.join(pkg_moveit, "config", "joint_limits.yaml"), context)
    controllers_file_path = _make_param_file(os.path.join(pkg_bringup, "config", "controllers.yaml"), context)
    sensors_3d_file_path = _make_param_file(os.path.join(pkg_bringup, "config", "sensors_3d.yaml"), context)
    moveit_controllers_file_path = _make_param_file(os.path.join(pkg_moveit, "config", "moveit_controllers.yaml"), context)

    robot_desc = (
        ParameterBuilder("group_a_description")
        .xacro_parameter(
            "robot_description",
            "urdf/group_a.urdf.xacro",
            mappings={
                "parent": parent_link,
                "xyz": xyz,
                "rpy": rpy,
                "lift_type": lift_type,
                "ur_type": ur_type,
                "sim_gazebo": sim_gazebo,
                "use_fake_hardware": use_fake_hardware,
                "simulation_controllers": str(controllers_file_path),
                "namespace": namespace,
            },
        )
        .to_dict()
    )

    gz_bridge_yaml_path = _make_param_file(os.path.join(pkg_bringup, "config", "gz_bridge.yaml"), context)

    sim_time_param = {"use_sim_time": LaunchConfiguration("sim_gazebo")}

    moveit_config = (
        MoveItConfigsBuilder(namespace, package_name="group_a_moveit_config")
        .robot_description(
            file_path=os.path.join(pkg_description, "urdf", "group_a.urdf.xacro"),
            mappings={
                "parent": parent_link,
                "xyz": xyz,
                "rpy": rpy,
                "lift_type": lift_type,
                "ur_type": ur_type,
                "sim_gazebo": sim_gazebo,
                "use_fake_hardware": use_fake_hardware,
                "simulation_controllers": str(controllers_file_path),
                "namespace": namespace,
            },
        )
        .robot_description_semantic(file_path=os.path.join(pkg_description, "config", "combined_system.srdf.xacro"), mappings={"namespace": namespace})
        .robot_description_kinematics(os.path.join(pkg_moveit, "config", "kinematics.yaml"))
        .joint_limits(str(joint_limits_file_path))
        .trajectory_execution(str(controllers_file_path))
        .planning_scene_monitor(
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
            publish_planning_scene=True,
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .sensors_3d(str(sensors_3d_file_path))
        .planning_pipelines("ompl", ["ompl", "chomp", "stomp", "pilz_industrial_motion_planner"])
        .pilz_cartesian_limits(os.path.join(pkg_moveit, "config", "pilz_cartesian_limits.yaml"))
        .to_moveit_configs()
        .to_dict()
    )

    # ── 1. Robot State Publisher ─────────────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        namespace=namespace,
        parameters=[
            robot_desc,
            sim_time_param,
        ],
    )

    # ── 2. Standalone Controller Manager (real hardware only) ─────────────────
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        namespace=namespace,
        parameters=[
            robot_desc,
            controllers_file_path,
            sim_time_param,
        ],
        condition=UnlessCondition(LaunchConfiguration("sim_gazebo")),
        remappings=[("/robot_description", f"{namespace}/robot_description")],
    )

    # ── 3. Gazebo Spawner ────────────────────────────────────────────────────
    gazebo_spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        namespace=namespace,
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            namespace,
        ],
        condition=IfCondition(LaunchConfiguration("sim_gazebo")),
    )

    # ── 4. Camera Bridge ─────────────────────────────────────────────────────
    gz_default_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_bridge",
        output="screen",
        parameters=[sim_time_param],
        namespace=namespace,
        arguments=["--ros-args", "-p", f"config_file:={gz_bridge_yaml_path}"],
        condition=IfCondition(LaunchConfiguration("sim_gazebo")),
    )

    # ── 4b. Depth-to-Pointcloud (Bypasses buggy Gazebo RGBD pointcloud) ──────
    # https://docs.ros.org/en/rolling/p/depth_image_proc/doc/components.html
    depth_to_pointcloud_container = ComposableNodeContainer(
        name="depth_to_pointcloud_container",
        namespace=namespace,
        package="rclcpp_components",
        executable="component_container",
        output="screen",
        composable_node_descriptions=[
            # 1. Ευθυγράμμιση του Raw Depth με την RGB Κάμερα
            ComposableNode(
                package="depth_image_proc",
                plugin="depth_image_proc::RegisterNode",
                name="depth_register_node",
                parameters=[sim_time_param,{"approximate_sync": True}],
                namespace=namespace,
                remappings=[
                    ("depth/image_rect", "camera/depth"),
                    ("depth/camera_info", "camera/camera_info"),
                    ("rgb/camera_info", "camera/camera_info"),
                    ("depth_registered/camera_info", "camera/depth_registered/camera_info"),
                    ("depth_registered/image_rect", "camera/depth_registered/image_rect"),
                ],
            ),
            # 2. Δημιουργία του XYZRGB Point Cloud από τα ευθυγραμμισμένα δεδομένα
            ComposableNode(
                package="depth_image_proc",
                plugin="depth_image_proc::PointCloudXyzrgbNode",
                name="point_cloud_xyzrgb_node",
                parameters=[sim_time_param,{"approximate_sync": True}],
                namespace=namespace,
                remappings=[
                    ("depth_registered/image_rect", "camera/depth_registered/image_rect"),
                    ("rgb/image_rect_color", "camera/color"),
                    ("rgb/camera_info", "camera/camera_info"),
                    ("points", "camera/depth_registered/points"),
                ],
            ),
        ],
    )

    # ── 5. Controller Spawners ───────────────────────────────────────────────
    motion_default_active_controllers_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        namespace=namespace,
        arguments=[
            "joint_state_broadcaster",
            "all_joint_trajectory_controller",
            "--controller-manager",
            f"/{namespace}/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
        parameters=[sim_time_param],
    )

    motion_default_inactive_controllers_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        namespace=namespace,
        arguments=[
            "lift_joint_trajectory_controller",
            "ur_joint_trajectory_controller",
            "--inactive",
            "--controller-manager",
            f"/{namespace}/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
        parameters=[sim_time_param],
    )

    # ── 6. MoveIt move_group ─────────────────────────────────────────────────
    # Parameter loading order matters: last entry wins on key conflicts.
    #   - planning_params (dict, scalars only) → overrides any autogen pipeline keys
    #   - sensors_tmp_path (file path string)  → ROS 2 reads list params from file
    #   - octomap scalars dict                 → simple key/value, safe as dict

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        namespace=namespace,
        parameters=[
            moveit_config,
            moveit_controllers_file_path,
            sim_time_param,
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
        ],
        remappings=[
            ("/robot_description", f"{namespace}/robot_description"),
            ("/robot_description_semantic", f"{namespace}/robot_description_semantic"),
        ],
    )

    return [
        robot_state_publisher,
        controller_manager_node,
        gazebo_spawn_robot,
        gz_default_bridge,
        depth_to_pointcloud_container,
        move_group_node,
        TimerAction(
            period=4.0,
            actions=[motion_default_active_controllers_spawner],
        ),
        TimerAction(
            period=4.0,
            actions=[motion_default_inactive_controllers_spawner],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            *get_launch_arguments(),
            OpaqueFunction(function=launch_setup),
        ]
    )
