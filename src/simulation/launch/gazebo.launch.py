import os

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    simulation_pkg_description = get_package_share_directory('simulation')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([simulation_pkg_description, 'worlds', 'drone_world.sdf']),
        description='Gazebo world file to load',
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [TextSubstitution(text='-r '), LaunchConfiguration('world')]
        }.items(),
    )

    # Spawn robots in Gazebo

    spawn_arg = DeclareLaunchArgument(
        'spawn_drone',
        default_value='true',
        description='Whether to spawn the drone in Gazebo',
    )
    spawn_drone = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulation_pkg_description, 'launch', 'spawn_drone.launch.py')
        ),
        launch_arguments={'namespace': "drone"}.items(),
        condition=IfCondition(LaunchConfiguration('spawn_drone'))
    )

    spawn_oh_my_hans_arg = DeclareLaunchArgument(
        'spawn_oh_my_hans',
        default_value='true',
        description='Whether to spawn Oh My Hans in Gazebo',
    )
    spawn_oh_my_hans = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulation_pkg_description, 'launch', 'spawn_oh_my_hans.launch.py')
        ),
        launch_arguments={'namespace': "oh_my_hans"}.items(),
        condition=IfCondition(LaunchConfiguration('spawn_oh_my_hans'))
    )

    get_package_share_directory('ros_gz_bridge')
    bridge_params = os.path.join(simulation_pkg_description, 'config', 'gz_bridge.yaml')
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}'],
    )

    return LaunchDescription([
        world_arg, 
        spawn_arg, 
        spawn_oh_my_hans_arg,
        gazebo, 
        spawn_drone, 
        spawn_oh_my_hans,
        ros_gz_bridge,
    ])