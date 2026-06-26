import os
from typing import List, Tuple
from launch import Action, LaunchDescription
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import isaac_ros_launch_utils as lu

from nvblox_ros_python_utils.nvblox_launch_utils import NvbloxMode
from nvblox_ros_python_utils.nvblox_constants import NVBLOX_CONTAINER_NAME

def get_depth_image_remappings(mode: NvbloxMode) -> List[Tuple[str, str]]:
    remappings = [
        ('camera_0/depth/image', '/group_a/camera/depth/image_raw'),
        ('camera_0/depth/camera_info', '/group_a/camera/color/camera_info'),
        ('pose', '/group_a/pose')
    ]
    
    if mode is NvbloxMode.people_segmentation:
        remappings.extend([
            ('camera_0/color/image', '/camera0/segmentation/image_resized'),
            ('camera_0/color/camera_info', '/camera0/segmentation/camera_info_resized'),
            ('camera_0/mask/image', '/camera0/segmentation/people_mask'),
            ('camera_0/mask/camera_info', '/camera0/segmentation/camera_info_resized')
        ])
    else:
        remappings.extend([
            ('camera_0/color/image', '/group_a/camera/color/image_raw'),
            ('camera_0/color/camera_info', '/group_a/camera/color/camera_info')
        ])
        if mode is NvbloxMode.people_detection:
            remappings.extend([
                ('camera_0/mask/image', '/camera0/detection/people_mask'),
                ('camera_0/mask/camera_info', '/group_a/camera/color/camera_info')
            ])
    return remappings


def get_pointcloud_remappings() -> List[Tuple[str, str]]:
    return [
        ('pointcloud', '/group_a/camera/depth/points'),
        ('pose', '/group_a/pose')
    ]


def add_nvblox(args: lu.ArgumentContainer) -> List[Action]:
    mode = NvbloxMode(NvbloxMode[args.mode]) 
    input_type = args.input_type

    vision_share = get_package_share_directory('vision')
    base_config = os.path.join(vision_share, 'config', 'nvblox_params.yaml')
    
    if input_type == 'depth_image':
        remappings = get_depth_image_remappings(mode)  
        use_lidar = False
    elif input_type == 'pointcloud':
        assert mode not in [NvbloxMode.people_segmentation, NvbloxMode.people_detection], \
            "People segmentation/detection modes are built for 2D 'depth_image' inputs."
        remappings = get_pointcloud_remappings()
        use_lidar = True
    else:
        raise Exception(f"Invalid input_type: '{input_type}'. Choose 'depth_image' or 'pointcloud'.")
    
    parameters = [
        base_config,
        {'num_cameras': 1},
        {'use_lidar': use_lidar}
    ]

    if args.use_lidar_motion_compensation != '':
        parameters.append({'use_lidar_motion_compensation': lu.is_true(args.use_lidar_motion_compensation)})

    nvblox_node = ComposableNode(
        name='nvblox_node',
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode',
        remappings=remappings,
        parameters=parameters,
    )

    actions = []
    if lu.is_true(args.run_standalone):
        actions.append(lu.component_container(args.container_name))
        
    actions.extend([
        lu.load_composable_nodes(args.container_name, [nvblox_node]),
        lu.log_info(["Starting nvblox with input pipeline: '", str(input_type), "' in '", str(mode), "' mode."])
    ])
    return actions


def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('mode', 'static', description='nvblox mode: static, dynamic, people_segmentation, people_detection')
    args.add_arg('input_type', 'depth_image', description='Input pipeline: choose depth_image or pointcloud')
    args.add_arg('container_name', NVBLOX_CONTAINER_NAME)
    args.add_arg('run_standalone', 'True')
    args.add_arg('use_lidar_motion_compensation', '')

    args.add_opaque_function(add_nvblox)
    return LaunchDescription(args.get_launch_actions())