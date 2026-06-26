import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    AppendEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    # -------------------------------------------------------------------------
    # 1. Εντοπισμός Πακέτων
    # -------------------------------------------------------------------------
    simulation_pkg = get_package_share_directory("simulation")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    ur_desc_share = get_package_share_directory("ur_description")
    ewellix_desc_share = get_package_share_directory("ewellix_description")

    # -------------------------------------------------------------------------
    # 2. Ρύθμιση Περιβάλλοντος Gazebo (Resource Paths)
    # -------------------------------------------------------------------------
    gz_resource_paths = os.path.dirname(ur_desc_share) + ":" + os.path.dirname(ewellix_desc_share) + ":" + os.path.dirname(simulation_pkg)
    set_gz_resource_path = AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", gz_resource_paths)

    # -------------------------------------------------------------------------
    # 3. Launch Arguments
    # -------------------------------------------------------------------------
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=PathJoinSubstitution([simulation_pkg, "worlds", "drone_world.sdf"]),
        description="Gazebo world file to load",
    )

    spawn_group_a_arg = DeclareLaunchArgument(
        "spawn_group_a",
        default_value="true",
        description="Spawn Group A (Ewellix + UR + Orbbec) with ros2_control",
    )

    # -------------------------------------------------------------------------
    # 4. Gazebo
    # -------------------------------------------------------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": ["-r ", LaunchConfiguration("world")]}.items(),
    )

    # -------------------------------------------------------------------------
    # 5. Robot spawners
    # -------------------------------------------------------------------------

    # Group A: combined Ewellix + UR + Orbbec with full ros2_control
    spawn_group_a = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(simulation_pkg, "launch", "inc", "spawn_gz.group_a.launch.py")),
        condition=IfCondition(LaunchConfiguration("spawn_group_a")),
    )

    # -------------------------------------------------------------------------
    # 6. Global ROS2↔Gazebo bridge (clock)
    # -------------------------------------------------------------------------
    bridge_params = os.path.join(simulation_pkg, "config", "gz_bridge.yaml")
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=["--ros-args", "-p", f"config_file:={bridge_params}"],
    )

    # -------------------------------------------------------------------------
    # 6b. Επιπλέον ROS2↔Gazebo bridge ειδικά για το Group A (Υπό συνθήκη)
    # -------------------------------------------------------------------------
    bridge_group_a_params = os.path.join(simulation_pkg, "config", "gz_bridge.group_a.yaml")
    ros_gz_bridge_group_a = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="group_a_bridge",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=["--ros-args", "-p", f"config_file:={bridge_group_a_params}"],
        # Η γέφυρα αυτή θα εκκινήσει ΜΟΝΟ αν το spawn_group_a είναι true
        condition=IfCondition(LaunchConfiguration("spawn_group_a")),
    )

    # -------------------------------------------------------------------------
    # 7. Launch Description
    # -------------------------------------------------------------------------
    return LaunchDescription(
        [
            set_gz_resource_path,
            world_arg,
            spawn_group_a_arg,
            gazebo,
            spawn_group_a,
            ros_gz_bridge,
            ros_gz_bridge_group_a,
        ]
    )
