from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    joy_topic = LaunchConfiguration('joy_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    position_target_topic = LaunchConfiguration('position_target_topic')
    control_mode_topic = LaunchConfiguration('control_mode_topic')
    publish_rate_hz = LaunchConfiguration('publish_rate_hz')
    control_mode_service = LaunchConfiguration('control_mode_service')
    position_target_service = LaunchConfiguration('position_target_service')

    return LaunchDescription([
        DeclareLaunchArgument(
            'joy_topic',
            default_value='/joy',
            description='Joy output topic for the GUI publisher',
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/drone/odometry',
            description='Odometry topic for telemetry display',
        ),
        DeclareLaunchArgument(
            'position_target_topic',
            default_value='/drone/position_target',
            description='Current hold target topic for telemetry display',
        ),
        DeclareLaunchArgument(
            'control_mode_topic',
            default_value='/drone/control_mode',
            description='Current control mode topic for the GUI',
        ),
        DeclareLaunchArgument(
            'publish_rate_hz',
            default_value='20.0',
            description='Joy publish rate in Hz',
        ),
        DeclareLaunchArgument(
            'control_mode_service',
            default_value='/drone/set_control_mode',
            description='Control mode service for the GUI',
        ),
        DeclareLaunchArgument(
            'position_target_service',
            default_value='/drone/set_position_target',
            description='Position target service for the GUI',
        ),
        Node(
            package='drone_control',
            executable='joy_gui_publisher',
            name='joy_gui_publisher',
            output='screen',
            parameters=[{
                'joy_topic': joy_topic,
                'odom_topic': odom_topic,
                'position_target_topic': position_target_topic,
                'control_mode_topic': control_mode_topic,
                'publish_rate_hz': publish_rate_hz,
                'control_mode_service': control_mode_service,
                'position_target_service': position_target_service,
            }],
        ),
    ])