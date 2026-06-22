from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def _load_params_file(path):
    with open(path, 'r', encoding='utf-8') as stream:
        data = yaml.safe_load(stream) or {}

    if isinstance(data, dict) and len(data) == 1:
        root_key = next(iter(data))
        root_value = data[root_key]
        if isinstance(root_value, dict) and 'ros__parameters' in root_value:
            return root_value['ros__parameters'] or {}

    if isinstance(data, dict) and 'ros__parameters' in data:
        return data['ros__parameters'] or {}

    return data if isinstance(data, dict) else {}


def generate_launch_description():
    imu_topic = LaunchConfiguration('imu_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    joy_topic = LaunchConfiguration('joy_topic')
    motor_topic = LaunchConfiguration('motor_topic')
    namespace = LaunchConfiguration('namespace')

    pkg_share = get_package_share_directory('drone_control')
    config_dir = os.path.join(pkg_share, 'config')
    common_yaml = os.path.join(config_dir, 'common_params.yaml')
    attitude_yaml = os.path.join(config_dir, 'attitude_params.yaml')
    position_yaml = os.path.join(config_dir, 'position_params.yaml')
    rate_yaml = os.path.join(config_dir, 'rate_params.yaml')

    common_params = _load_params_file(common_yaml)
    attitude_params = _load_params_file(attitude_yaml)
    position_params = _load_params_file(position_yaml)
    rate_params = _load_params_file(rate_yaml)

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='drone',
            description='Robot namespace for the controller node',
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/drone/imu',
            description='Input IMU topic',
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/drone/odometry',
            description='Input odometry topic',
        ),
        DeclareLaunchArgument(
            'joy_topic',
            default_value='/joy',
            description='Input joystick topic, typically sensor_msgs/msg/Joy',
        ),
        DeclareLaunchArgument(
            'motor_topic',
            default_value='/drone/command/motor_speed',
            description='Output motor command topic',
        ),
        Node(
            package='drone_control',
            executable='drone_controller',
            name='drone_controller',
            namespace=namespace,
            output='screen',
            parameters=[common_params, attitude_params, position_params, rate_params, {
                # allow overrides from launch args
                'imu_topic': imu_topic,
                'odom_topic': odom_topic,
                'joy_topic': joy_topic,
                'motor_topic': motor_topic,
            }],
        ),
    ])