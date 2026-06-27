import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription, 
    GroupAction, 
    AppendEnvironmentVariable,
    TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    ld = LaunchDescription()

    # 1. Εντοπισμός Πακέτων για το Gazebo Global Environment
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_ur_desc = get_package_share_directory("ur_description")
    pkg_ewellix_desc = get_package_share_directory("ewellix_description")
    pkg_workcell_bringup = get_package_share_directory("workcell_bringup")
    pkg_workcell_description = get_package_share_directory("workcell_description")

    # 2. Global Gazebo Resource Paths (GZ_SIM_RESOURCE_PATH)
    gz_resource_paths = (
        os.path.dirname(pkg_ur_desc) + ":" + 
        os.path.dirname(pkg_ewellix_desc) + ":" +
        os.path.dirname(pkg_workcell_bringup) + ":" +
        os.path.dirname(pkg_workcell_description)
    )
    set_gz_resource_path = AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", gz_resource_paths)
    ld.add_action(set_gz_resource_path)

    # 3. Global Launch Arguments
    use_fake_hardware_arg = DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="true",
        description="True for mock components (RViz only). False for Gazebo or Real Hardware.",
    )

    sim_gazebo_arg = DeclareLaunchArgument(
        "sim_gazebo",
        default_value="false",
        description="True to launch Gazebo Simulator.",
    )
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=PathJoinSubstitution([pkg_workcell_description, "worlds", "workcell_world.sdf"]),
        description="Gazebo world file to load",
    )

    # 4. Εκκίνηση Global Gazebo Instance
    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args": [
                "-r ",  # trailing space
                LaunchConfiguration("world")
            ]
        }.items(),
        condition=IfCondition(LaunchConfiguration("sim_gazebo")),
    )
    ld.add_action(gazebo)

    # 5. Global Clock Bridge (ROS 2 <-> Gazebo time synchronization)
    bridge_params = os.path.join(pkg_workcell_bringup, "config", "gz_bridge.yaml")
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=["--ros-args", "-p", f"config_file:={bridge_params}"],
        condition=IfCondition(LaunchConfiguration("sim_gazebo")),
    )
    ld.add_action(clock_bridge)

    # 6. Global Static Transform Publisher για το World Frame
    world_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
    )
    ld.add_action(world_node)

    # 7. Ορισμός των Ρομπότ στην Κυψέλη Εργασίας
    robots_config = [
        {'name': 'robot_1','xyz': '0.0 0.0 0.0','rpy': '0.0 0.0 0.0'},
        {'name': 'robot_2', 'xyz': '1.0 0.0 0.0', 'rpy': '0.0 0.0 3.14159'},
        # {'name': 'robot_3', 'xyz': '1.0 1.0 0.0', 'rpy': '0.0 0.0 3.14159'},
        # {'name': 'robot_4', 'xyz': '0.0 1.0 0.0', 'rpy': '0.0 0.0 3.14159'}
    ]

    pkg_group_a_bringup_share = get_package_share_directory('group_a_bringup')
    group_a_launch_path = os.path.join(pkg_group_a_bringup_share, 'launch', 'bringup.launch.py')

    # 8. Loop που καλεί το ανεξάρτητο bringup του κάθε ρομπότ
    for i, robot in enumerate(robots_config):
        robot_stack = GroupAction(
            actions=[
                PushRosNamespace(robot['name']),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(group_a_launch_path),
                    launch_arguments={
                        'parent_link': 'world',
                        'xyz': robot['xyz'],
                        'rpy': robot['rpy'],
                        'sim_gazebo': LaunchConfiguration("sim_gazebo"),
                        'use_fake_hardware': LaunchConfiguration("use_fake_hardware")
                    }.items()
                ),
            ]
        )
        # Stagger each robot by 0.5s to avoid simultaneous Gazebo spawn requests
        ld.add_action(TimerAction(period=float(i) * 0.5, actions=[robot_stack]))

    return LaunchDescription([
        use_fake_hardware_arg,
        sim_gazebo_arg,
        world_arg,
        ld
    ])
