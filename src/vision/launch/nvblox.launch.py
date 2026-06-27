import os
from typing import List, Tuple
from launch import Action, LaunchDescription
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import isaac_ros_launch_utils as lu

from nvblox_ros_python_utils.nvblox_launch_utils import NvbloxMode
from nvblox_ros_python_utils.nvblox_constants import NVBLOX_CONTAINER_NAME


# ── Workcell Robot Registry ───────────────────────────────────────────────────
# Single source of truth for all robots in the workcell.
# Add/remove robots here and the nvblox remappings update automatically.
WORKCELL_ROBOTS = [
    "robot_1",
    "robot_2",
    "robot_3",
    "robot_4",
]


def get_depth_image_remappings(
    mode: NvbloxMode,
    robots: List[str],
) -> List[Tuple[str, str]]:
    """
    Build camera remappings for all robots in the workcell.
    Each robot's camera_{i} maps to its namespaced ROS topics.

    Example for robot_1 as camera_0:
        camera_0/depth/image  →  /robot_1/camera/depth/image_raw
        camera_0/color/image  →  /robot_1/camera/color/image_raw
        ...
    """
    remappings = []

    for i, robot_ns in enumerate(robots):
        cam = f"camera_{i}"

        # Depth
        remappings.append((f"{cam}/depth/image",       f"/{robot_ns}/camera/depth/image_raw"))
        remappings.append((f"{cam}/depth/camera_info", f"/{robot_ns}/camera/color/camera_info"))

        # Pose — each robot provides its own pose (wrist camera moves with arm)
        remappings.append((f"{cam}/pose", f"/{robot_ns}/pose"))

        if mode is NvbloxMode.people_segmentation:
            remappings.extend([
                (f"{cam}/color/image",       f"/{robot_ns}/segmentation/image_resized"),
                (f"{cam}/color/camera_info", f"/{robot_ns}/segmentation/camera_info_resized"),
                (f"{cam}/mask/image",        f"/{robot_ns}/segmentation/people_mask"),
                (f"{cam}/mask/camera_info",  f"/{robot_ns}/segmentation/camera_info_resized"),
            ])
        else:
            remappings.extend([
                (f"{cam}/color/image",       f"/{robot_ns}/camera/color/image_raw"),
                (f"{cam}/color/camera_info", f"/{robot_ns}/camera/color/camera_info"),
            ])
            if mode is NvbloxMode.people_detection:
                remappings.extend([
                    (f"{cam}/mask/image",        f"/{robot_ns}/detection/people_mask"),
                    (f"{cam}/mask/camera_info",  f"/{robot_ns}/camera/color/camera_info"),
                ])

    return remappings


def get_pointcloud_remappings(robots: List[str]) -> List[Tuple[str, str]]:
    """
    Build pointcloud remappings for all robots.
    Each robot contributes a separate pointcloud stream.
    """
    remappings = []
    for i, robot_ns in enumerate(robots):
        cam = f"camera_{i}"
        remappings.append((f"{cam}/pointcloud", f"/{robot_ns}/camera/depth/points"))
        remappings.append((f"{cam}/pose",       f"/{robot_ns}/pose"))
    return remappings


def add_nvblox(args: lu.ArgumentContainer) -> List[Action]:
    mode = NvbloxMode(NvbloxMode[args.mode])
    input_type = args.input_type

    # ── Resolve robot list ────────────────────────────────────────────────────
    # Allow override via launch arg; fall back to workcell registry
    if hasattr(args, "robots") and args.robots:
        robots = [r.strip() for r in args.robots.split(",")]
    else:
        robots = WORKCELL_ROBOTS

    num_cameras = len(robots)

    # ── Config ────────────────────────────────────────────────────────────────
    vision_share = get_package_share_directory("vision")
    base_config = os.path.join(vision_share, "config", "nvblox_params.yaml")

    # ── Remappings ────────────────────────────────────────────────────────────
    if input_type == "depth_image":
        remappings = get_depth_image_remappings(mode, robots)
        use_lidar = False
    elif input_type == "pointcloud":
        assert mode not in [NvbloxMode.people_segmentation, NvbloxMode.people_detection], (
            "People segmentation/detection modes require 'depth_image' input, not 'pointcloud'."
        )
        remappings = get_pointcloud_remappings(robots)
        use_lidar = True
    else:
        raise ValueError(
            f"Invalid input_type: '{input_type}'. Choose 'depth_image' or 'pointcloud'."
        )

    # ── Parameters ────────────────────────────────────────────────────────────
    parameters = [
        base_config,
        {"num_cameras": num_cameras},   # auto-derived from robot count
        {"use_lidar": use_lidar},
    ]

    if args.use_lidar_motion_compensation != "":
        parameters.append(
            {"use_lidar_motion_compensation": lu.is_true(args.use_lidar_motion_compensation)}
        )

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

    actions.extend([
        lu.load_composable_nodes(args.container_name, [nvblox_node]),
        lu.log_info([
            "Starting shared workcell nvblox | ",
            f"input: '{input_type}' | ",
            f"mode: '{mode}' | ",
            f"cameras: {num_cameras} ({', '.join(robots)})",
        ]),
    ])
    return actions


def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg(
        "mode", "static",
        description="nvblox mode: static, dynamic, people_segmentation, people_detection",
    )
    args.add_arg(
        "input_type", "depth_image",
        description="Input pipeline: depth_image or pointcloud",
    )
    args.add_arg(
        "robots", "",
        description=(
            "Comma-separated robot namespaces to include as nvblox cameras. "
            "Defaults to all robots in WORKCELL_ROBOTS if empty. "
            "Example: 'robot_1,robot_2'"
        ),
    )
    args.add_arg("container_name", NVBLOX_CONTAINER_NAME)
    args.add_arg("run_standalone", "True")
    args.add_arg("use_lidar_motion_compensation", "")

    args.add_opaque_function(add_nvblox)
    return LaunchDescription(args.get_launch_actions())