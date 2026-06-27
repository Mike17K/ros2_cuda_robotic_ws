import os
import yaml
from typing import Any, cast
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition
from launch_ros.actions import Node
import xacro
import tempfile

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

    def launch_setup(context):
        pkg_description = get_package_share_directory("group_a_description")
        pkg_control = get_package_share_directory("group_a_control")
        pkg_moveit = get_package_share_directory("group_a_moveit_config") 
        pkg_bringup = get_package_share_directory("group_a_bringup") 

        # Ανάκτηση τιμών runtime
        use_fake_hardware = LaunchConfiguration("use_fake_hardware").perform(context)
        sim_gazebo = LaunchConfiguration("sim_gazebo").perform(context)
        lift_type = LaunchConfiguration("lift_type").perform(context)
        ur_type = LaunchConfiguration("ur_type").perform(context)
        parent_link = LaunchConfiguration("parent_link").perform(context)
        xyz = LaunchConfiguration("xyz").perform(context)
        rpy = LaunchConfiguration("rpy").perform(context)

        current_namespace = context.launch_configurations.get('ros_namespace', '')
        runtime_namespace = current_namespace if current_namespace else "group_a"
        runtime_namespace = runtime_namespace.strip("/")  # Αφαίρεση αρχικών και τελικών "/"
        print(f"Current ROS Namespace: '{current_namespace}'")

        xacro_file = os.path.join(pkg_description, "urdf", "group_a.urdf.xacro")
        
        # Φόρτωση του νέου τοπικού αρχείου σχεδιασμού κίνησης
        local_planning_yaml = os.path.join(pkg_bringup, "config", "planning.yaml")

        # Επεξεργασία Xacro
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
                    "namespace": runtime_namespace,
                },
            ),
        )

        robot_desc = {"robot_description": robot_description_config.toxml()}

        # Φόρτωση MoveIt SRDF
        srdf_file = os.path.join(pkg_moveit, "config", "combined_system.srdf")
        with open(srdf_file, "r") as f:
            robot_desc_semantic = {"robot_description_semantic": f.read()}

        # Δυναμική φόρτωση Kinematics (Από το autogen MoveIt πακέτο)
        kinematics_file = os.path.join(pkg_moveit, "config", "kinematics.yaml")
        with open(kinematics_file, "r") as f:
            raw_kinematics = yaml.safe_load(f)
        kinematics_params = {"robot_description_kinematics": raw_kinematics}

        # Δυναμική φόρτωση Joint Limits (Από το autogen MoveIt πακέτο)
        joint_limits_file = os.path.join(pkg_moveit, "config", "joint_limits.yaml")
        with open(joint_limits_file, "r") as f:
            raw_joint_limits = yaml.safe_load(f)
        joint_limits_params = {"robot_description_planning": raw_joint_limits}

        # 1. Robot State Publisher
        robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_desc],
        )


        controllers_yaml = os.path.join(pkg_control, "config", "group_a_controllers.yaml")

        # 2. Standalone Controller Manager
        controller_manager_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            parameters=[robot_desc, controllers_yaml],
            condition=UnlessCondition(LaunchConfiguration("sim_gazebo")),
        )

        # 3. Gazebo Spawner Node
        gazebo_spawn_robot = Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=[
                "-topic", "robot_description",
                "-name", current_namespace if current_namespace else "group_a_robot",
                "-x", xyz.split()[0], "-y", xyz.split()[1], "-z", xyz.split()[2],
                "-R", rpy.split()[0], "-P", rpy.split()[1], "-Y", rpy.split()[2],
            ],
            condition=IfCondition(LaunchConfiguration("sim_gazebo")),
        )

        # ── Απόλυτη και Ασφαλής Διαδρομή για το Παραγόμενο YAML ───────────────────
        template_bridge_yaml = os.path.join(pkg_bringup, "config", "gz_bridge.yaml")
        
        generated_bridge_yaml = f"/tmp/{runtime_namespace}_gz_bridge.yaml"
        
        with open(template_bridge_yaml, "r") as f:
            bridge_content = f.read()
        print(f"Generating bridge YAML for namespace '{runtime_namespace}' at '{generated_bridge_yaml}'")
        bridge_content = bridge_content.replace("{namespace}", runtime_namespace)
        
        with open(generated_bridge_yaml, "w") as f:
            f.write(bridge_content)

        camera_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="camera_bridge",
            output="screen",
            parameters=[{"use_sim_time": True}],
            arguments=["--ros-args", "-p", f"config_file:={generated_bridge_yaml}"],
            condition=IfCondition(LaunchConfiguration("sim_gazebo")),
        )

        # 5. Controllers Spawners
        motion_default_active_controllers_spawner = Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[
                "joint_state_broadcaster",
                "lift_joint_trajectory_controller",
                "ur_joint_trajectory_controller",
                "--controller-manager", f"/{runtime_namespace}/controller_manager",
            ],
        )

        local_planning_yaml_path = os.path.join(pkg_bringup, "config", "planning.yaml")
        with open(local_planning_yaml_path, "r") as f:
            local_planning_yaml = yaml.safe_load(f)

        # Flatten: extract just ros__parameters for move_group
        planning_params = local_planning_yaml.get("move_group", {}).get("ros__parameters", {})

        # 6. MoveIt Node
        move_group_node = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                robot_desc,
                robot_desc_semantic,
                kinematics_params,
                joint_limits_params,
                {"use_sim_time": LaunchConfiguration("sim_gazebo")},
                planning_params,
            ],
            # Fixes controller_manager 'Waiting for data on robot_description' hang:
            # Remaps the global check from /group_a/robot_description to its explicit namespace context
            remappings=[
                ("/robot_description", f"{runtime_namespace}/robot_description")
            ]
        )

        return [
            robot_state_publisher,
            controller_manager_node,
            gazebo_spawn_robot,
            camera_bridge,
            move_group_node,
            TimerAction(
                period=4.0,
                actions=[motion_default_active_controllers_spawner]
            ),
        ]

    return LaunchDescription(
        [
            use_fake_hardware_arg,
            sim_gazebo_arg,
            lift_type_arg,
            ur_type_arg,
            parent_link_arg,
            xyz_arg,
            rpy_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )