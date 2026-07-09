import os
import tempfile
from typing import Any
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterFile
from launch_param_builder import ParameterBuilder

def get_launch_arguments() -> list[DeclareLaunchArgument]:
    args = []
    args.append(DeclareLaunchArgument("use_fake_hardware", default_value="true", description="Use mock_components/GenericSystem (true) or real hardware drivers (false)"))
    args.append(DeclareLaunchArgument("sim_gazebo", default_value="false", description="Switch to true if launching inside a Gazebo Simulation environment"))
    args.append(DeclareLaunchArgument("lift_type", default_value="ur_620", description="Ewellix model type"))
    args.append(DeclareLaunchArgument("ur_type", default_value="ur10", description="UR robot type"))
    args.append(DeclareLaunchArgument("parent_link", default_value="world", description="Parent link in the workcell"))
    args.append(DeclareLaunchArgument("xyz", default_value="0.0 0.0 0.0", description="Robot spawn position"))
    args.append(DeclareLaunchArgument("rpy", default_value="0.0 0.0 0.0", description="Robot spawn orientation"))
    args.append(DeclareLaunchArgument("namespace", default_value="", description="Namespace for the robot tf frames, topics and nodes"))
    args.append(DeclareLaunchArgument("tf_prefix", default_value="", description="Prefix for all TF frames after namespace is applied"))
    return args

# Reference registry to keep temporary runtime objects alive during nodes lifespan
_runtime_file_refs: list[Any] = []

def _make_param_file(path, context):
    pf = ParameterFile(path, allow_substs=True)
    _runtime_file_refs.append(pf)  # prevent garbage collection / early temp-file deletion
    return pf.evaluate(context)

def launch_setup(context):
    pkg_bringup = get_package_share_directory("planning_bringup")
    pkg_cumotion = get_package_share_directory("isaac_ros_cumotion")

    # ── 1. Runtime Configurations ───────────────────────────────────────────
    use_fake_hardware = LaunchConfiguration("use_fake_hardware").perform(context)
    sim_gazebo = LaunchConfiguration("sim_gazebo").perform(context)
    lift_type = LaunchConfiguration("lift_type").perform(context)
    ur_type = LaunchConfiguration("ur_type").perform(context)
    parent_link = LaunchConfiguration("parent_link").perform(context)
    xyz = LaunchConfiguration("xyz").perform(context)
    rpy = LaunchConfiguration("rpy").perform(context)
    namespace = LaunchConfiguration("namespace").perform(context)
    tf_prefix = LaunchConfiguration("tf_prefix").perform(context)

    # Load dynamic, namespace-substituted parameters file
    cumotion_params_file_path = _make_param_file(os.path.join(pkg_bringup, "config", "group_a","cumotion_params.yaml"), context)

    # ── 2. Construct Dynamic URDF via ParameterBuilder ──────────────────────
    robot_desc_dict = (
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
                # "simulation_controllers": str(cumotion_params_file_path), # Fallback mapping placeholder # this is not needed
                "namespace": namespace,
                "tf_prefix": tf_prefix,
            },
        )
        .to_dict()
    )
    
    # Save processed URDF contents into a tracked transient file
    urdf_content = robot_desc_dict["robot_description"]
    temp_urdf = tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False)
    temp_urdf.write(urdf_content)
    temp_urdf.close()
    _runtime_file_refs.append(temp_urdf)

    # ── 3. Parse and Substitute XRDF Template ───────────────────────────────
    xrdf_template_path = os.path.join(pkg_bringup, "config", "group_a", "group_a.xrdf")
    
    with open(xrdf_template_path, "r") as f:
        xrdf_content = f.read()

    # Apply active namespace and tf_prefix formatting rules matching your Xacro trees
    processed_xrdf = xrdf_content.replace("{namespace}", namespace).replace("{tf_prefix}", tf_prefix)

    # Write processed configuration out to a safe running environment path
    temp_xrdf = tempfile.NamedTemporaryFile(mode="w", suffix=".xrdf", delete=False)
    temp_xrdf.write(processed_xrdf)
    temp_xrdf.close()
    _runtime_file_refs.append(temp_xrdf)

    # ── 4. Launch Isaac ROS CuMotion Pipeline ───────────────────────────────
    cumotion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_cumotion, "launch", "isaac_ros_cumotion.launch.py"])
        ),
        launch_arguments={
            "cumotion_action_server.xrdf_file_path": temp_xrdf.name,
            "cumotion_action_server.urdf_file_path": temp_urdf.name,
            "cumotion_action_server.parameters_path": str(cumotion_params_file_path),
        }.items(),
    )

    return [cumotion_launch]

def generate_launch_description():
    return LaunchDescription(
        [
            *get_launch_arguments(),
            OpaqueFunction(function=launch_setup),
        ]
    )