Automated tuning sweep

This folder contains `tune_sweep.py` which performs a simple parameter sweep against a running simulator + `drone_controller` node.

Prerequisites

- Source your ROS 2 workspace and the simulator environment so `ros2` and ROS2 Python APIs are available.
- Start the simulator and the `drone_controller` node (use the provided launch file which reads YAML config files).
- Ensure `sensor_msgs/msg/Imu` is published at `/drone/imu` and controller listens on `/joy`.

Quick start

1. Make the script executable:

```bash
chmod +x src/drone_control/scripts/tune_sweep.py
```

2. Run the script while the sim and controller are running:

```bash
./src/drone_control/scripts/tune_sweep.py --duration 4 --target_deg 6
```

Output

- `tune_results.csv` will contain ranked trial results.
- The script prints recommended `ros2 param set` commands for the best parameter set.

Notes & caveats

- The script is intentionally simple: it drives the controller via joystick steps and measures IMU roll/pitch.
- For more realistic sweeps, increase the grid ranges, change the step shape, or use trajectory-following and odometry instead of IMU only.
- Running this script requires a real-time sim run; it does not start the simulator for you.
