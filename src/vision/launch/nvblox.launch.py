import os
from typing import List, Tuple
import yaml
from launch import Action, LaunchDescription
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import isaac_ros_launch_utils as lu

from nvblox_ros_python_utils.nvblox_launch_utils import NvbloxMode
from nvblox_ros_python_utils.nvblox_constants import NVBLOX_CONTAINER_NAME


def get_depth_image_remappings(
    mode: NvbloxMode,
    depth_topics: List[str],
    depth_info_topics: List[str],
    color_topics: List[str],
    color_info_topics: List[str],
    pose_topics: List[str] = [],
) -> List[Tuple[str, str]]:
    """Build remappings using the loaded topic arrays index-by-index."""
    remappings = []

    for i, (depth, depth_info, color, color_info, pose) in enumerate(zip(depth_topics, depth_info_topics, color_topics, color_info_topics, pose_topics)):
        cam = f"camera_{i}"
        remappings.extend(
            [
                (f"{cam}/depth/image", depth),
                (f"{cam}/depth/camera_info", depth_info),
                (f"pose", pose),
            ]
        )

        # If people_segmentation mode is active, override with the segmentation pipeline targets
        if mode is NvbloxMode.people_segmentation:
            img_target, info_target = "/segmentation/image_resized", "/segmentation/camera_info_resized"
        else:
            img_target, info_target = color, color_info

        remappings.extend(
            [
                (f"{cam}/color/image", img_target),
                (f"{cam}/color/camera_info", info_target),
            ]
        )

    return remappings


def add_nvblox(args: lu.ArgumentContainer) -> List[Action]:
    if args.input_type != "depth_image":
        raise ValueError(f"Invalid input_type: '{args.input_type}'. Choose 'depth_image' or 'pointcloud'.")

    mode = NvbloxMode(NvbloxMode[args.mode])
    vision_share = get_package_share_directory("vision")

    # ── Parse YAML Topics ─────────────────────────────────────────────────────
    topics_config_path = os.path.join(vision_share, "config", "nvblox_topics.yaml")
    if not os.path.exists(topics_config_path):
        raise FileNotFoundError(f"Topics configuration file not found at: {topics_config_path}")

    with open(topics_config_path, "r") as f:
        yaml_data = yaml.safe_load(f)

    try:
        params = yaml_data["/**"]["ros__parameters"]
        depth_image_topics = params.get("depth_image_topics", [])
        depth_info_topics = params.get("depth_info_topics", [])
        color_image_topics = params.get("color_image_topics", [])
        color_info_topics = params.get("color_info_topics", [])
        pose_topics = params.get("pose_topics", [])
    except (KeyError, TypeError):
        raise KeyError("Invalid layout in nvblox_topics.yaml. Must match '/**' -> 'ros__parameters'.")

    # ── Configuration & Parameters ────────────────────────────────────────────
    num_cameras = len(depth_image_topics)
    remappings = get_depth_image_remappings(mode, depth_image_topics, depth_info_topics, color_image_topics, color_info_topics, pose_topics)

    parameters = [
        os.path.join(vision_share, "config", "nvblox_params.yaml"),
        {"num_cameras": num_cameras},
        {"use_lidar": False},
        {"use_sim_time": lu.is_true(args.use_sim_time)},
    ]

    if args.use_lidar_motion_compensation != "":
        parameters.append({"use_lidar_motion_compensation": lu.is_true(args.use_lidar_motion_compensation)})

    # ── Node & Actions Assembly ───────────────────────────────────────────────
    nvblox_node = ComposableNode(
        name="nvblox_node",
        package="nvblox_ros",
        plugin="nvblox::NvbloxNode",
        remappings=remappings,
        parameters=parameters,
    )

    actions = []
    if lu.is_true(args.run_standalone):
        actions.append(lu.component_container(args.container_name))

    actions.extend(
        [
            lu.load_composable_nodes(args.container_name, [nvblox_node]),
            lu.log_info(
                [
                    "Starting explicit nvblox pipeline | ",
                    f"input: '{args.input_type}' | ",
                    f"mode: '{mode}' | ",
                    f"cameras: {num_cameras}",
                ]
            ),
        ]
    )
    return actions


def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg("mode", "static", description="nvblox mode: static, dynamic, people_segmentation, people_detection")
    args.add_arg("input_type", "depth_image", description="Input pipeline: depth_image or pointcloud")
    args.add_arg("container_name", NVBLOX_CONTAINER_NAME)
    args.add_arg("run_standalone", "True")
    args.add_arg("use_lidar_motion_compensation", "")
    args.add_arg("use_sim_time", "True", description="Use simulation clock")

    args.add_opaque_function(add_nvblox)
    return LaunchDescription(args.get_launch_actions())
