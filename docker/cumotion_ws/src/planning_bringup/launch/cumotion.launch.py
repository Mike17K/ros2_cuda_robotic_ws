import os
import tempfile
import yaml
from typing import Any, Optional
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile
from launch_param_builder import ParameterBuilder

_param_file_refs: list[Any] = []


def _make_param_file(path, context):
    pf = ParameterFile(path, allow_substs=True)
    _param_file_refs.append(pf)  # prevent garbage collection / temp-file deletion
    return pf.evaluate(context)


def _make_temp_file(contents: str) -> str:
    """Create a temporary file with the given contents and return its path."""
    temp_file = tempfile.NamedTemporaryFile(delete=False, mode="w", suffix=".yaml")
    temp_file.write(contents)
    temp_file.close()
    return temp_file.name


def read_params(pkg_name, params_dir, params_file_name):
    params_file = os.path.join(
        get_package_share_directory(pkg_name), params_dir, params_file_name)
    return yaml.safe_load(open(params_file, 'r'))


def launch_args_from_params(pkg_name, params_dir, params_file_name, prefix: Optional[str] = None):
    launch_args = []
    launch_configs = {}
    params = read_params(pkg_name, params_dir, params_file_name)

    for param, value in params['/**']['ros__parameters'].items():
        if value is not None:
            arg_name = param if prefix is None else f'{prefix}.{param}'
            launch_args.append(DeclareLaunchArgument(name=arg_name, default_value=str(value)))
            launch_configs[param] = LaunchConfiguration(arg_name)

    return launch_args, launch_configs


def get_launch_arguments() -> list[DeclareLaunchArgument]:
    args = []
    args.append(DeclareLaunchArgument("use_fake_hardware", default_value="true"))
    args.append(DeclareLaunchArgument("sim_gazebo", default_value="false"))
    args.append(DeclareLaunchArgument("lift_type", default_value="ur_620"))
    args.append(DeclareLaunchArgument("ur_type", default_value="ur10"))
    args.append(DeclareLaunchArgument("parent_link", default_value="world"))
    args.append(DeclareLaunchArgument("xyz", default_value="0.0 0.0 0.0"))
    args.append(DeclareLaunchArgument("rpy", default_value="0.0 0.0 0.0"))
    args.append(DeclareLaunchArgument("namespace", default_value="robot_1"))
    args.append(DeclareLaunchArgument("tf_prefix", default_value=""))
    return args


def launch_setup(context, launch_configs):
    pkg_bringup = get_package_share_directory("planning_bringup")

    use_fake_hardware = LaunchConfiguration("use_fake_hardware").perform(context)
    sim_gazebo = LaunchConfiguration("sim_gazebo").perform(context)
    lift_type = LaunchConfiguration("lift_type").perform(context)
    ur_type = LaunchConfiguration("ur_type").perform(context)
    parent_link = LaunchConfiguration("parent_link").perform(context)
    xyz = LaunchConfiguration("xyz").perform(context)
    rpy = LaunchConfiguration("rpy").perform(context)
    namespace = LaunchConfiguration("namespace").perform(context)
    tf_prefix = LaunchConfiguration("tf_prefix").perform(context)

    robot_desc_dict = (
        ParameterBuilder("group_a_description")
        .xacro_parameter(
            "robot_description",
            "urdf/group_a.urdf.xacro",
            mappings={
                "parent": parent_link, "xyz": xyz, "rpy": rpy,
                "lift_type": lift_type, "ur_type": ur_type,
                "sim_gazebo": sim_gazebo, "use_fake_hardware": use_fake_hardware,
                "namespace": namespace, "tf_prefix": tf_prefix,
            },
        )
        .to_dict()
    )

    urdf_path_sub = _make_temp_file(robot_desc_dict["robot_description"])

    xrdf_path = _make_param_file(
        PathJoinSubstitution([
            get_package_share_directory("planning_bringup"), "config", "group_a", "group_a.xrdf"
        ]),
        context
    )

    launch_configs['xrdf_file_path'] = str(xrdf_path)
    launch_configs['urdf_file_path'] = str(urdf_path_sub)
    launch_configs['parameters_path'] = str(os.path.join(pkg_bringup, "config", "group_a", "cumotion_params.yaml"))
    launch_configs['joint_states_topic'] = str("/" + namespace + "/joint_states")

    env_variables = dict(os.environ)

    # launch_configs['enable_cuda_mps'] is a LaunchConfiguration substitution,
    # not a bool -- it must be resolved via perform(context) before branching.
    enable_cuda_mps = launch_configs['enable_cuda_mps'].perform(context)
    if enable_cuda_mps.lower() == 'true':
        env_variables.update({
            'CUDA_MPS_ACTIVE_THREAD_PERCENTAGE':
                launch_configs['cuda_mps_active_thread_percentage'].perform(context),
            'CUDA_MPS_PIPE_DIRECTORY':
                launch_configs['cuda_mps_pipe_directory'].perform(context),
            'CUDA_MPS_CLIENT_PRIORITY':
                launch_configs['cuda_mps_client_priority'].perform(context),
        })

    # Static planning scene server
    static_planning_scene_server = ComposableNode(
        name='static_planning_scene_server',
        package='isaac_ros_cumotion',
        plugin='nvidia::isaac_ros::cumotion::StaticPlanningSceneServer',
        namespace=namespace,
        parameters=[{
            'moveit_collision_objects_scene_file':
                launch_configs['moveit_collision_objects_scene_file']
        }],
    )

    cumotion_planner_node = ComposableNode(
        name='cumotion_planner',
        package='isaac_ros_cumotion',
        plugin='nvidia::isaac_ros::cumotion::CumotionPlanner',
        namespace=namespace,
        parameters=[
            launch_configs
        ],
    )

    cumotion_container = ComposableNodeContainer(
        name='cumotion_container',
        namespace=namespace,
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            static_planning_scene_server,
            cumotion_planner_node,
        ],
        output='screen',
        env=env_variables,
    )

    return [cumotion_container]


def generate_launch_description():
    cumotion_launch_args, cumotion_launch_configs = launch_args_from_params(
        'planning_bringup',
        'config/group_a',
        'cumotion_params.yaml',
        'cumotion_action_server',
    )

    return LaunchDescription([
        *get_launch_arguments(),
        *cumotion_launch_args,
        OpaqueFunction(
            function=launch_setup,
            kwargs={'launch_configs': cumotion_launch_configs},
        ),
    ])