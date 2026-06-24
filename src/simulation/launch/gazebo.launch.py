import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    simulation_pkg_description = get_package_share_directory('simulation')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    ur_desc_share = get_package_share_directory('ur_description')
    ur_desc_parent = os.path.dirname(ur_desc_share)
    set_gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        ur_desc_parent
    )

    # 1. Ορισμός των Launch Configurations
    world_config = LaunchConfiguration('world')
    spawn_drone_config = LaunchConfiguration('spawn_drone')
    spawn_oh_my_hans_config = LaunchConfiguration('spawn_oh_my_hans')
    spawn_ur10_config = LaunchConfiguration('spawn_ur10')

    # 2. Δήλωση των Launch Arguments με τις προεπιλεγμένες τιμές τους
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([simulation_pkg_description, 'worlds', 'drone_world.sdf']),
        description='Gazebo world file to load',
    )

    spawn_arg = DeclareLaunchArgument(
        'spawn_drone',
        default_value='true',
        description='Whether to spawn the drone in Gazebo',
    )

    spawn_oh_my_hans_arg = DeclareLaunchArgument(
        'spawn_oh_my_hans',
        default_value='true',
        description='Whether to spawn Oh My Hans in Gazebo',
    )

    spawn_ur10_arg = DeclareLaunchArgument(
        'spawn_ur10',
        default_value='true',
        description='Whether to spawn UR10 in Gazebo',
    )

    # 3. Συγχώνευση του flag '-r ' με τη διαδρομή του κόσμου
    # Το ROS 2 launch επιτρέπει λίστα από substitutions για να τα ενώσει αυτόματα σε ένα string
    gz_args_value = ['-r ', world_config]

    # 4. Συμπερίληψη του Gazebo Launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': gz_args_value
        }.items(),
    )

    # 5. Ρύθμιση των Spawners των ρομπότ
    spawn_drone = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulation_pkg_description, 'launch', 'spawn_drone.launch.py')
        ),
        launch_arguments={'namespace': "drone"}.items(),
        condition=IfCondition(spawn_drone_config)
    )

    spawn_oh_my_hans = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulation_pkg_description, 'launch', 'spawn_oh_my_hans.launch.py')
        ),
        launch_arguments={'namespace': "oh_my_hans"}.items(),
        condition=IfCondition(spawn_oh_my_hans_config)
    )

    spawn_ur10 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulation_pkg_description, 'launch', 'spawn_ur.launch.py')
        ),
        launch_arguments={'namespace': "ur10", 'ur_type': "ur10"}.items(),
        condition=IfCondition(spawn_ur10_config)
    )

    # 6. Ρύθμιση της Γέφυρας Επικοινωνίας (ROS <-> Gazebo)
    bridge_params = os.path.join(simulation_pkg_description, 'config', 'gz_bridge.yaml')
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}'],
    )

    # Επιστροφή όλων των αντικειμένων με τη σωστή σειρά
    return LaunchDescription([
        set_gz_resource_path, 
        world_arg, 
        spawn_arg, 
        spawn_oh_my_hans_arg,
        spawn_ur10_arg,
        gazebo, 
        spawn_drone, 
        spawn_oh_my_hans,
        spawn_ur10,
        ros_gz_bridge,
    ])
