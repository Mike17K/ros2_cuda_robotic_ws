import os
from typing import List, Tuple
from launch import Action, LaunchDescription
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import isaac_ros_launch_utils as lu

from nvblox_ros_python_utils.nvblox_launch_utils import NvbloxMode
from nvblox_ros_python_utils.nvblox_constants import NVBLOX_CONTAINER_NAME

# ── Explicit Topic Configurations ─────────────────────────────────────────────
# Define your topics directly here. The length of these lists determines the
# number of cameras passed to nvblox.

POINTCLOUD_TOPICS = [
    "/robot_1/camera/depth/points",
]

DEPTH_IMAGE_TOPICS = [
    # "/camera_0/depth/image_raw",
]

DEPTH_INFO_TOPICS = [
    # "/camera_0/depth/camera_info",
]

# Note: nvblox expects camera topics using indexed names (camera_0, camera_1, etc.)
# and remaps your actual topics to them.


def get_depth_image_remappings(
    mode: NvbloxMode,
    depth_topics: List[str],
    info_topics: List[str],
) -> List[Tuple[str, str]]:
    """Build remappings using the explicit depth topic lists."""
    remappings = []

    for i, (depth_topic, info_topic) in enumerate(zip(depth_topics, info_topics)):
        cam = f"camera_{i}"

        remappings.append((f"{cam}/depth/image", depth_topic))
        remappings.append((f"{cam}/depth/camera_info", info_topic))

        # Note: If your system uses specific color or mask topics, map them explicitly below:
        if mode is NvbloxMode.people_segmentation:
            # Placeholder example if color/mask is needed for segmentation
            remappings.extend(
                [
                    (f"{cam}/color/image", "/segmentation/image_resized"),
                    (f"{cam}/color/camera_info", "/segmentation/camera_info_resized"),
                ]
            )
        else:
            remappings.extend(
                [
                    (f"{cam}/color/image", "/camera/color/image_raw"),
                    (f"{cam}/color/camera_info", "/camera/color/camera_info"),
                ]
            )

    return remappings


def get_pointcloud_remappings(pc_topics: List[str]) -> List[Tuple[str, str]]:
    """Build remappings using the explicit pointcloud topic list."""
    remappings = []
    for i, pc_topic in enumerate(pc_topics):
        cam = f"camera_{i}"
        remappings.append((f"{cam}/pointcloud", pc_topic))
    return remappings


def add_nvblox(args: lu.ArgumentContainer) -> List[Action]:
    mode = NvbloxMode(NvbloxMode[args.mode])
    input_type = args.input_type

    # ── Remappings & Camera Count ─────────────────────────────────────────────
    if input_type == "depth_image":
        num_cameras = len(DEPTH_IMAGE_TOPICS)
        remappings = get_depth_image_remappings(mode, DEPTH_IMAGE_TOPICS, DEPTH_INFO_TOPICS)
        use_lidar = False
    elif input_type == "pointcloud":
        assert mode not in [NvbloxMode.people_segmentation, NvbloxMode.people_detection], "People segmentation/detection modes require 'depth_image' input, not 'pointcloud'."
        num_cameras = len(POINTCLOUD_TOPICS)
        remappings = get_pointcloud_remappings(POINTCLOUD_TOPICS)
        use_lidar = True
    else:
        raise ValueError(f"Invalid input_type: '{input_type}'. Choose 'depth_image' or 'pointcloud'.")

    # ── Config ────────────────────────────────────────────────────────────────
    vision_share = get_package_share_directory("vision")
    base_config = os.path.join(vision_share, "config", "nvblox_params.yaml")

    # ── Parameters ────────────────────────────────────────────────────────────
    parameters = [
        base_config,
        {"num_cameras": num_cameras},
        {"use_lidar": use_lidar},
        {"use_sim_time": lu.is_true(args.use_sim_time)},
    ]

    if args.use_lidar_motion_compensation != "":
        parameters.append({"use_lidar_motion_compensation": lu.is_true(args.use_lidar_motion_compensation)})

    # ── Node ──────────────────────────────────────────────────────────────────
    nvblox_node = ComposableNode(
        name="nvblox_node",
        package="nvblox_ros",
        plugin="nvblox::NvbloxNode",
        remappings=remappings,
        parameters=parameters,
    )

    # ── Actions ───────────────────────────────────────────────────────────────
    actions = []
    if lu.is_true(args.run_standalone):
        actions.append(lu.component_container(args.container_name))

    actions.extend(
        [
            lu.load_composable_nodes(args.container_name, [nvblox_node]),
            lu.log_info(
                [
                    "Starting explicit nvblox pipeline | ",
                    f"input: '{input_type}' | ",
                    f"mode: '{mode}' | ",
                    f"cameras: {num_cameras}",
                ]
            ),
        ]
    )
    return actions


def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg(
        "mode",
        "static",
        description="nvblox mode: static, dynamic, people_segmentation, people_detection",
    )
    args.add_arg(
        "input_type",
        "pointcloud",
        description="Input pipeline: depth_image or pointcloud",
    )
    args.add_arg("container_name", NVBLOX_CONTAINER_NAME)
    args.add_arg("run_standalone", "True")
    args.add_arg("use_lidar_motion_compensation", "")
    args.add_arg("use_sim_time", "True", description="Use simulation clock")

    args.add_opaque_function(add_nvblox)
    return LaunchDescription(args.get_launch_actions())
