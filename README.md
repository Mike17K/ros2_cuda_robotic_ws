# Setup

```
uv init --python=3.11
```

# Cuda

https://nvidia-isaac-ros.github.io/getting_started/index.html
https://nvlabs.github.io/curobo/latest/getting-started/installation.html
https://nvidia-isaac.github.io/nvblox/v0.0.10/index.html
https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/index.html#quickstart

CUDA 13

```bash
sudo apt install nvidia-driver-580
sudo reboot
```

# Build

```
make
```

# Usefull reference

/opt/ros/jazzy/share/nvblox/nvblox_examples_bringup/

# Trubleshooting

add this in .bashrc

```
export UV_SKIP_WHEEL_FILENAME_CHECK=1
```
