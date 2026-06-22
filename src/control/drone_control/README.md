# drone_control

This package turns joystick and IMU data into motor speed commands for a quadrotor.

It is written for ROS 2 and uses a two-layer controller:

- Outer layer: geometric attitude control on SO(3)
- Inner layer: angular-rate PID control

If those terms are new to you, this README explains them from first principles.

## 1) What this package does

At every control tick (default 50 Hz), the controller:

1. Reads latest joystick input (`sensor_msgs/msg/Joy`)
2. Reads latest orientation + gyro rates from IMU (`sensor_msgs/msg/Imu`)
3. Computes desired body moments (roll, pitch, yaw torque-like commands)
4. Mixes those moments into 4 motor speeds
5. Publishes motor speeds (`actuator_msgs/msg/Actuators`)

If joystick times out or deadman is not pressed, it publishes zero/min motor command.

## 2) ROS 2 basics you need

ROS 2 components used here:

- Node: a running process. Here the main node is `drone_controller`.
- Topic: named data stream. Nodes publish or subscribe.
- Message type: schema for topic data.
- Parameter: runtime config value (gain, topic name, limits, etc).
- Launch file: script that starts node(s) with arguments and parameters.

Main topics:

- IMU input: `/drone/imu` (default)
- Joystick input: `/joy` (default)
- Motor command output: `/drone/command/motor_speed` (default)

## 3) Why two control loops

A quadrotor is hard to stabilize because orientation dynamics are fast.
A common and practical structure is:

- Outer loop (slower conceptually): converts orientation error to desired angular rates
- Inner loop (fast): tracks angular rates using PID

This split is more robust than trying to do everything in one simple controller.

The package now exposes two high-level control modes:

- `manual_attitude`: the joystick directly commands roll, pitch, throttle, and yaw rate
- `position_hold`: the joystick nudges x, y, z, and yaw targets, and the controller closes the loop using odometry + IMU
- `position_hold`: the controller closes the loop using odometry + IMU; joystick nudges horizontal position and yaw targets, while altitude is held by a separate PID loop

## 4) SO(3) in simple words

### 4.1 What SO(3) means

SO(3) is the set of all 3D rotations.

A rotation matrix R is in SO(3) if:

- R^T R = I (orthonormal)
- det(R) = 1

Why use this here:

- It avoids Euler-angle singularities (gimbal lock)
- It gives a clean, global way to compute orientation error
- It is standard in modern quadrotor control

### 4.2 How orientation error is computed

Current orientation from IMU quaternion is converted to rotation matrix R.
Desired rotation matrix is built from joystick commands:

- roll command -> Rx(phi)
- pitch command -> Ry(theta)
- yaw command -> Rz(psi)

Desired matrix:

Rd = Rz(psi*d) * Ry(theta*d) * Rx(phi_d)

Error matrix (skew-symmetric part):

E = 0.5 _ (Rd^T _ R - R^T \* Rd)

e_R (vector form) is obtained using the vee map from E.

Then desired body rates are generated:

- omega_des_x = K_R_attitude \* e_R_x
- omega_des_y = K_R_attitude \* e_R_y
- omega_des_z = K_R_yaw \* yaw_error

### 4.3 What the gains mean

- `K_R_attitude`: how strongly roll/pitch angle error becomes desired roll/pitch rate
- `K_R_yaw`: how strongly yaw angle error becomes desired yaw rate

Bigger gain -> faster correction but more oscillation risk.

When yaw is configured as a rate command, the roll/pitch reference is anchored to the current yaw frame so the aircraft does not lose roll/pitch authority after large yaw turns.

In `position_hold`, the horizontal position error is converted into roll/pitch targets in the current yaw frame, and yaw target error is converted into a yaw-rate command.

## 5) Inner-rate PID loop

The controller reads gyro rates omega from IMU and computes:

rate_error = omega - omega_des

For each axis, PID gives moment-like command M:

M = -(Kp*e + Ki*integral(e) + Kd\*de/dt) + cross_term

`cross_term` approximates rigid-body coupling due to inertia and rotation.

Then moments are scaled to motor-delta space with:

- `torque_to_speed_gain`

## 6) Motor mixing

For quad with 4 motors:

- Start from base speed (hover + throttle trim)
- Add roll/pitch/yaw deltas by fixed sign pattern

In code order:

- m0: +roll +pitch -yaw
- m1: -roll +pitch +yaw
- m2: -roll -pitch -yaw
- m3: +roll -pitch +yaw

Finally each motor is clamped to:

- `min_motor_speed` .. `max_motor_speed`

## 7) Safety behavior

Safety checks before control output:

- Deadman button must be pressed (`deadman_button`)
- Joystick must be recent (`joy_timeout_sec`)

If either fails, motor command is set to minimum for all motors.

## 8) Configuration files

The launch file loads four YAML files:

- `config/common_params.yaml`
- `config/attitude_params.yaml`
- `config/position_params.yaml`
- `config/rate_params.yaml`

### 8.1 Common parameters (`common_params.yaml`)

- `imu_topic`: IMU topic name
- `odom_topic`: odometry topic name used by position-hold mode
- `joy_topic`: joystick topic name
- `motor_topic`: motor output topic name
- `publish_rate_hz`: control loop rate
- `motor_count`: expected number of motors (optimized for 4)
- `min_motor_speed`: lower clamp
- `max_motor_speed`: upper clamp
- `hover_motor_speed`: baseline speed around hover
- `throttle_trim_range`: max add/subtract around hover from throttle stick
- `throttle_deadzone`: small throttle region treated as zero
- `joy_timeout_sec`: max allowed age of joystick message
- `deadman_button`: index in Joy buttons array
- `axis_roll`: index in Joy axes array
- `axis_pitch`: index in Joy axes array
- `axis_yaw`: index in Joy axes array
- `axis_throttle`: index in Joy axes array
- `mode_toggle_button`: Joy button used to switch between manual and position-hold modes
- `mode_reset_button`: Joy button used to recenter the position target to the current vehicle state

The controller also subscribes to `odom_topic` for position-hold mode. Default: `/drone/odometry`.

### 8.2 Attitude/SO(3) parameters (`attitude_params.yaml`)

- `attitude_inertia`: Ixx and Iyy estimate (kg\*m^2)
- `attitude_inertia_z`: Izz estimate (kg\*m^2)
- `K_R_attitude`: roll/pitch orientation-to-rate gain
- `K_R_yaw`: yaw orientation-to-rate gain
- `max_attitude_deg`: max commanded roll/pitch from full stick
- `yaw_gain`: fallback yaw stick scaling when IMU orientation is unavailable
- `torque_to_speed_gain`: scales computed moments to motor speed deltas
- `max_yaw_rate_deg`: maximum yaw rate used by the yaw-rate controller

### 8.3 Position / target parameters (`position_params.yaml`)

- `position_target_step_xy_m`: target nudge per control tick for x/y in position mode
- `position_target_step_z_m`: target nudge for z when the target is adjusted directly
- `position_target_step_yaw_deg`: target nudge per control tick for yaw in position mode
- `position_xy_gain`: converts horizontal position error into roll/pitch targets
- `altitude_Kp`: proportional gain for altitude hold
- `altitude_Ki`: integral gain for altitude hold
- `altitude_Kd`: derivative gain for altitude hold
- `altitude_integral_limit`: cap on the altitude integral term
- `altitude_integral_leak_rate`: leak applied to the altitude integral term over time
- `max_position_tilt_deg`: cap on the roll/pitch target generated by position control

### 8.4 Rate PID parameters (`rate_params.yaml`)

Roll/pitch shared gains:

- `rate_Kp`
- `rate_Ki`
- `rate_Kd`

Yaw-specific gains:

- `rate_Kp_yaw`
- `rate_Ki_yaw`
- `rate_Kd_yaw`

Integral anti-windup settings:

- `rate_integral_limit`: maximum absolute size allowed for the stored integral term
- `rate_integral_leak_rate`: exponential decay rate applied to old integral error over time

## 9) Launch files

### `launch/drone_controller.launch.py`

Starts `drone_controller` with:

- namespace argument (`namespace`, default `drone`)
- optional topic overrides (`imu_topic`, `odom_topic`, `joy_topic`, `motor_topic`)
- parameters loaded from the four YAML files

Example:

```bash
ros2 launch drone_control drone_controller.launch.py
```

With overrides:

```bash
ros2 launch drone_control drone_controller.launch.py \
  namespace:=drone \
  imu_topic:=/drone/imu \
  odom_topic:=/drone/odometry \
  joy_topic:=/joy \
  motor_topic:=/drone/command/motor_speed
```

Service for changing control modes:

```bash
ros2 service call /drone/set_control_mode drone_control/srv/SetControlMode "{mode: 1}"
```

Mode values:

- `0` = `manual_attitude`
- `1` = `position_hold`

Service for setting a position target directly from the GUI:

```bash
ros2 service call /drone/set_position_target drone_control/srv/SetPositionTarget "{x: 0.0, y: 0.0, z: 1.5, yaw_deg: 90.0}"
```

The joy GUI now shows a position-target card with x/y/z/yaw fields, step values, and nudge buttons for position-hold tuning.

### `launch/joy_gui.launch.py`

Starts a small Tk GUI node that publishes `sensor_msgs/msg/Joy`.
Useful when you do not have a physical joystick.

The panel now also includes direct mode buttons for `manual_attitude` and `position_hold`, plus Joy pulse buttons for the controller's mode-toggle and target-reset inputs.

Keyboard shortcuts:

- `Space`: toggle deadman
- `M`: send the Joy mode-toggle pulse
- `R`: send the Joy target-reset pulse
- `0`: reset axes to neutral

```bash
ros2 launch drone_control joy_gui.launch.py
```

## 10) How to tune in practice

Suggested order:

1. Verify axis mapping and deadman first
2. Start in `manual_attitude` and confirm motor signs and yaw rate feel correct
3. Test `position_hold` with the vehicle fixed, then verify the target reset button and the service switch
4. Set conservative `hover_motor_speed`, clamps, and throttle range
5. Tune rate PID (`rate_*`) with small attitude commands
6. Tune geometric gains (`K_R_attitude`, `K_R_yaw`)
7. Tune `position_xy_gain`, `altitude_Kp`, and `max_position_tilt_deg` for position hold
8. Adjust `torque_to_speed_gain` to map moments to usable motor deltas

Symptoms and likely fixes:

- Slow response: increase `rate_Kp` or `K_R_attitude`
- Oscillation: reduce `rate_Kp` and/or `K_R_attitude`, increase `rate_Kd` slightly
- Steady offset: increase `rate_Ki` carefully
- Integral builds up too much: lower `rate_integral_limit`
- Controller remembers old errors too long: raise `rate_integral_leak_rate`
- Yaw unstable: lower `K_R_yaw` or yaw PID gains
- Position hold drifts slowly: increase `position_xy_gain` or `altitude_Kp`
- Position hold feels too aggressive: lower `position_xy_gain`, `altitude_Kp`, or `max_position_tilt_deg`

## 11) Known assumptions and limits

- Mixer is explicitly for 4 motors.
- If `motor_count != 4`, controller publishes same base speed to all motors.
- Controller expects valid IMU quaternion + angular velocity.
- Parameters are updateable at runtime via ROS 2 parameter interface.

## 12) Source map

Main implementation:

- `src/drone_controller.cpp`

Support modules:

- `include/drone_control/utils.hpp`, `src/utils.cpp` (math helpers)
- `include/drone_control/rate_pid.hpp`, `src/rate_pid.cpp` (PID)
- `include/drone_control/mixer.hpp`, `src/mixer.cpp` (quad mixing)
- `srv/SetControlMode.srv` (service for switching manual/position modes)
- `config/position_params.yaml` (position-hold target and gain settings)

Python tools:

- `scripts/joy_gui_publisher.py` (GUI Joy publisher)

If you want, the next step can be adding a one-page tuning playbook with specific starting ranges for each parameter and a step-by-step test script.
