import os
from typing import Any, cast
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Δήλωση των Launch Arguments
    ns_arg = DeclareLaunchArgument('namespace', default_value='ur10', description='Robot namespace')
    ur_type_arg = DeclareLaunchArgument('ur_type', default_value='ur10', description='UR type (e.g., ur10, ur5e, ur3e)')
    
    x_pose = LaunchConfiguration('x', default='2.0')
    y_pose = LaunchConfiguration('y', default='2.0')
    z_pose = LaunchConfiguration('z', default='0.0')

    def launch_setup(context, *args, **kwargs):
        pkg_description = get_package_share_directory('ur_description')
        robot_namespace = LaunchConfiguration('namespace').perform(context)
        ur_type = LaunchConfiguration('ur_type').perform(context)
        
        # Στόχευση του επίσημου κεντρικού xacro αρχείου των Universal Robots
        xacro_file = os.path.join(pkg_description, 'urdf', 'ur.urdf.xacro')
        
        # Ορισμός των απαραίτητων mappings για το ur_description και το Gazebo
        robot_description_config = cast(
            Any,
            xacro.process_file(
                xacro_file,
                mappings={
                    'ur_type': ur_type,
                    'sim_ignition': 'true',          # Ενεργοποιεί το plugin για το Gazebo Sim (πρώην Ignition)
                    'simulation_controllers': '',    # Απαραίτητο για το simulation interface
                    'tf_prefix': f'{robot_namespace}_',
                    'name': robot_namespace,         # Αυτό λύνει το "Undefined substitution argument name"!
                },
            ),
        )

        robot_desc = {'robot_description': robot_description_config.toxml()}

        # Κόμβος Robot State Publisher
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            namespace=robot_namespace,
            parameters=[
                robot_desc,
                {'use_sim_time': True},
                {'frame_prefix': f'{robot_namespace}/'},
            ],
            remappings=[
                ('/joint_states', f'/{robot_namespace}/joint_states')
            ]
        )

        # Κόμβος δημιουργίας/εισαγωγής της οντότητας στο Gazebo Sim (ros_gz_sim)
        spawn_ur = Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                '-string', robot_description_config.toxml(),
                '-name', robot_namespace,
                '-allow_renaming', 'true',
                '-x', x_pose,
                '-y', y_pose,
                '-z', z_pose,
            ]
        )

        return [
            robot_state_publisher, 
            spawn_ur,
        ]

    return LaunchDescription([
        ns_arg,
        ur_type_arg,
        OpaqueFunction(function=launch_setup),
    ])
