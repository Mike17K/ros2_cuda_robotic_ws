from isaac_ros_launch_utils.all_types import *
import isaac_ros_launch_utils as lu
import os
from ament_index_python.packages import get_package_share_directory

from nvblox_ros_python_utils.nvblox_launch_utils import NvbloxMode, NvbloxCamera
from nvblox_ros_python_utils.nvblox_constants import NVBLOX_CONTAINER_NAME

def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    
    # Ρυθμίσεις Container της NVIDIA
    args.add_arg('container_name', NVBLOX_CONTAINER_NAME, description='Name of the component container.')
    args.add_arg('log_level', 'info', choices=['debug', 'info', 'warn'], cli=True)
    args.add_arg('use_sim_time', 'True', description='Use simulation time')

    actions = args.get_launch_actions()

    # Καθολική ενεργοποίηση του Sim Time
    actions.append(SetParameter('use_sim_time', True))

    # Εντοπισμός του share directory του πακέτου 'vision'
    pkg_share = get_package_share_directory('vision')

    # Ορισμός των Remappings σε JSON string μορφή που απαιτεί το launch_arguments
    remap_dict = {
        '/camera/depth/image_rect_raw': '/camera/depth_image',
        '/camera/depth/camera_info': '/camera/camera_info'
    }

    # 1. Κλήση του εσωτερικού Nvblox Launch της NVIDIA
    actions.append(
        lu.include(
            'nvblox_examples_bringup',
            'launch/perception/nvblox.launch.py',
            launch_arguments={
                'container_name': args.container_name,
                'mode': str(NvbloxMode.static),       
                'camera': str(NvbloxCamera.realsense), 
                'num_cameras': 1,
                'parameters_path': os.path.join(pkg_share, 'config', 'nvblox_params.yaml'),
                # Περνάμε τα remappings σωστά μέσω της παραμέτρου της NVIDIA [1]
                'remap_dictionary': str(remap_dict)
            }
        )
    )

    # 2. Εκκίνηση του Container της NVIDIA που θα φιλοξενήσει το Node
    actions.append(
        Node(
            package='rclcpp_components',
            executable='component_container_mt',
            name=args.container_name,
            output='screen'
        )
    )

    return LaunchDescription(actions)
