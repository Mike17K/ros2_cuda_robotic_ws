<img src="docs/shared_nvblox.png">

# Diffusion Robot Test Workspace

ROS 2 (Jazzy) workspace for multi-arm manipulation on lift-mounted UR arms ("group_a": Ewellix lift + UR arm + Orbbec camera), planned with [Isaac ROS cuMotion](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_cumotion/isaac_ros_cumotion/index.html) against a shared [nvblox](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/index.html) reconstruction, simulated in Gazebo. Longer-term goal is a diffusion-policy planner — see [docs/DIFFUSION_MODEL_IDEA.md](docs/DIFFUSION_MODEL_IDEA.md).

All development happens inside a container built via the [Isaac ROS CLI](https://nvidia-isaac-ros.github.io/concepts/dev_env/index.html) — there is no host ROS install. Design rationale (why nvblox runs in static TSDF mode, why the workspace lives inside the container) is in [docs/STRUCTURAL_DESISIONS.md](docs/STRUCTURAL_DESISIONS.md).

## Layout

| Path | What |
|---|---|
| `src/workcell` | Gazebo world + shared workcell description |
| `src/robots/group_a` | Robot description + MoveIt config for group_a |
| `src/planning_bringup` | cuMotion planning launch/config |
| `src/vision` | nvblox launch/config |
| `src/isaac_ros_cumotion_fork` | Submodule, [Mike17K/isaac_ros_cumotion](https://github.com/Mike17K/isaac_ros_cumotion) |
| `Dockerfile.cumotion_ws` | Layer added on top of the Isaac ROS base image |
| `scripts/` | Entry points, see below |

## Prerequisites (host)

- [NVIDIA Container Toolkit](https://nvidia-isaac-ros.github.io/getting_started/index.html) + `isaac-ros-cli` — see `scripts/setup_host.sh`
- `docker login nvcr.io` with an [NGC API key](https://org.ngc.nvidia.com/account/api-keys) (username: `$oauthtoken`)
- `isaac_ros_common` pinned to the `3.2-15` release

## Entry points (`scripts/`)

| Script | Purpose |
|---|---|
| `build_docker_image.sh` | Builds/activates the container (`isaac-ros activate --build-local`) |
| `shell.sh` | Opens a shell in the running container |
| `entrypoint.sh` | Container entrypoint, runs `make` |
| `setup_workspace.sh` | First-boot dependency install inside the container |
| `setup_host.sh` | One-off host setup (NVIDIA container toolkit + isaac-ros-cli) |
| `launch/launch_ws.sh` | Opens a Terminator layout and launches workcell / cuMotion / RViz / nvblox panels |

## Build & run (inside the container)

```bash
make            # colcon build
make rosdeps    # install rosdep dependencies
make builds n=<package>   # build a single package
```

Then, e.g.:

```bash
ros2 launch workcell_bringup workcell.launch.py sim_gazebo:=true use_fake_hardware:=false
ros2 launch planning_bringup cumotion.launch.py
ros2 launch vision nvblox.launch.py
ros2 launch workcell_bringup rviz.launch.py rviz_namespace:=robot_1
```

## References

- [Isaac ROS dev environment](https://nvidia-isaac-ros.github.io/concepts/dev_env/index.html)
- [Isaac ROS getting started](https://nvidia-isaac-ros.github.io/getting_started/index.html)
- [Isaac ROS cuMotion](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_cumotion/isaac_ros_cumotion/index.html)
- [Isaac ROS nvblox quickstart](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/index.html#quickstart)
- [nvblox](https://nvidia-isaac.github.io/nvblox/v0.0.10/index.html)
- [curobo](https://nvlabs.github.io/curobo/latest/getting-started/installation.html)
- [Isaac Sim quick install](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/quick-install.html#isaac-sim-quick-install)
- [Isaac Sim robot config generator (Lula)](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/robot_setup_tutorials/tutorial_generate_robot_config.html)
- [direnv](https://direnv.net/)
