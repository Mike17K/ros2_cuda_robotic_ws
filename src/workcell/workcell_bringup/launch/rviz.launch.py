import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Standalone RViz launch for the workcell.
    Run this AFTER workcell.launch.py is already up (move_group needs to be
    alive so RViz can query its parameters/services on startup).

    Example:
        ros2 launch workcell_bringup rviz.launch.py rviz_namespace:=robot_1
    """

    namespace_arg = DeclareLaunchArgument(
        "rviz_namespace",
        default_value="robot_1",
        description="Which robot's move_group namespace RViz's MotionPlanning display targets (robot_1 or robot_2)",
    )

    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(
            get_package_share_directory("workcell_bringup"), "config", "workcell.rviz"
        ),
        description="Path to RViz config file",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock",
    )

    def launch_setup(context):
        ns = LaunchConfiguration("rviz_namespace").perform(context)

        pkg_bringup = get_package_share_directory("workcell_bringup")
        pkg_moveit = get_package_share_directory("group_a_moveit_config")

        kinematics_file = os.path.join(pkg_moveit, "config", "kinematics.yaml")
        with open(kinematics_file, "r") as f:
            raw_kinematics = yaml.safe_load(f)
        kinematics_params = {"robot_description_kinematics": raw_kinematics}

        planning_yaml = os.path.join(pkg_bringup, "config", "planning.yaml")
        with open(planning_yaml, "r") as f:
            planning_params = yaml.safe_load(f)

        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            parameters=[
                kinematics_params,
                planning_params,
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
            remappings=[
                ("/robot_description", f"/{ns}/robot_description"),
                ("/robot_description_semantic", f"/{ns}/robot_description_semantic"),
                ("/robot_description_planning", f"/{ns}/robot_description_planning"),
                ("/robot_description_kinematics", f"/{ns}/robot_description_kinematics"),
            ],
        )

        return [rviz_node]

    return LaunchDescription(
        [
            namespace_arg,
            rviz_config_arg,
            use_sim_time_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )