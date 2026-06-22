import os
from typing import Any, cast
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    ns_arg = DeclareLaunchArgument('namespace', default_value='drone', description='Robot namespace')
    x_pose = LaunchConfiguration('x', default='0.0')
    y_pose = LaunchConfiguration('y', default='0.0')
    z_pose = LaunchConfiguration('z', default='0.5') # Spawn slightly above the ground

    def launch_setup(context, *args, **kwargs):
        pkg_description = get_package_share_directory('drone_description')
        robot_namespace = LaunchConfiguration('namespace').perform(context)
        xacro_file = os.path.join(pkg_description, 'description', 'drone.urdf.xacro')
        robot_description_config = cast(
            Any,
            xacro.process_file(
                xacro_file,
                mappings={'robot_namespace': robot_namespace},
            ),
        )
        robot_desc = {'robot_description': robot_description_config.toxml()}

        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            namespace=robot_namespace,
            parameters=[
                robot_desc,
                {'use_sim_time': True},
                { 'frame_prefix': f'/{robot_namespace}/' },
            ],
            remappings=[
                ('/joint_states', f'/{robot_namespace}/joint_states')
            ]
        )

        spawn_drone = Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                '-string', robot_description_config.toxml(),
                '-name', robot_namespace,
                '-allow_renaming', 'true',
                '-x', x_pose,
                '-y', y_pose,
                '-z', z_pose
            ]
        )

        return [
            robot_state_publisher, 
            spawn_drone,
        ]

    return LaunchDescription([
        ns_arg,
        OpaqueFunction(function=launch_setup),
    ])
