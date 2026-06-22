#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "actuator_msgs/msg/actuators.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "drone_control/utils.hpp"
#include "drone_control/rate_pid.hpp"
#include "drone_control/mixer.hpp"
#include "drone_control/srv/set_position_target.hpp"
#include "drone_control/srv/set_control_mode.hpp"

constexpr double kPi = 3.14159265358979323846;

class DroneController : public rclcpp::Node
{
public:
  enum class ControlMode : std::uint8_t
  {
    MANUAL_ATTITUDE = 0,
    POSITION_HOLD = 1,
  };

  explicit DroneController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("drone_controller", options)
  {
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/drone/imu");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/drone/odometry");
    joy_topic_ = declare_parameter<std::string>("joy_topic", "joy");
    motor_topic_ = declare_parameter<std::string>("motor_topic", "/drone/command/motor_speed");
    position_target_topic_ = declare_parameter<std::string>("position_target_topic", "/drone/position_target");
    control_mode_topic_ = declare_parameter<std::string>("control_mode_topic", "/drone/control_mode");

    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 50.0);
    motor_count_ = declare_parameter<int>("motor_count", 4);
    min_motor_speed_ = declare_parameter<double>("min_motor_speed", 0.0);
    max_motor_speed_ = declare_parameter<double>("max_motor_speed", 1000.0);
    hover_motor_speed_ = declare_parameter<double>("hover_motor_speed", 505.0);
    deadman_button_ = declare_parameter<int>("deadman_button", 4);
    joy_timeout_sec_ = declare_parameter<double>("joy_timeout_sec", 0.5);
    axis_roll_ = declare_parameter<int>("axis_roll", 0);
    axis_pitch_ = declare_parameter<int>("axis_pitch", 1);
    axis_yaw_ = declare_parameter<int>("axis_yaw", 2);
    axis_throttle_ = declare_parameter<int>("axis_throttle", 3);
    mode_toggle_button_ = declare_parameter<int>("mode_toggle_button", 6);
    mode_reset_button_ = declare_parameter<int>("mode_reset_button", 7);

    // tuned inertia consistent with drone_description geometry (~0.017 kg*m^2)
    inertia_ = declare_parameter<double>("attitude_inertia", 0.017);  // kg*m^2 (approx)
    torque_to_speed_gain_ = declare_parameter<double>("torque_to_speed_gain", 0.7);
    max_attitude_deg_ = declare_parameter<double>("max_attitude_deg", 15.0);
    yaw_gain_ = declare_parameter<double>("yaw_gain", 20.0); // fallback mapping when IMU unavailable
    max_yaw_rate_deg_ = declare_parameter<double>("max_yaw_rate_deg", 90.0); // joystick -> yaw rate (deg/s)
    throttle_trim_range_ = declare_parameter<double>("throttle_trim_range", 80.0);
    throttle_deadzone_ = declare_parameter<double>("throttle_deadzone", 0.08);
    attitude_inertia_z_ = declare_parameter<double>("attitude_inertia_z", inertia_);
    position_target_step_xy_m_ = declare_parameter<double>("position_target_step_xy_m", 0.10);
    position_target_step_z_m_ = declare_parameter<double>("position_target_step_z_m", 0.05);
    position_target_step_yaw_deg_ = declare_parameter<double>("position_target_step_yaw_deg", 10.0);
    position_xy_gain_ = declare_parameter<double>("position_xy_gain", 0.35);
    max_position_tilt_deg_ = declare_parameter<double>("max_position_tilt_deg", max_attitude_deg_);
    altitude_Kp_ = declare_parameter<double>("altitude_Kp", 120.0);
    altitude_Ki_ = declare_parameter<double>("altitude_Ki", 0.0);
    altitude_Kd_ = declare_parameter<double>("altitude_Kd", 0.0);
    altitude_integral_limit_ = declare_parameter<double>("altitude_integral_limit", 2000.0);
    altitude_integral_leak_rate_ = declare_parameter<double>("altitude_integral_leak_rate", 1.0);

    // Geometric controller gains (SO(3))
    K_R_attitude_ = declare_parameter<double>("K_R_attitude", 8.0); // attitude gain (roll/pitch)
    K_R_yaw_ = declare_parameter<double>("K_R_yaw", 4.0); // yaw attitude gain

    // Rate PID inner-loop gains (roll/pitch share, yaw separate)
    rate_Kp_ = declare_parameter<double>("rate_Kp", 6.0);
    rate_Ki_ = declare_parameter<double>("rate_Ki", 0.1);
    rate_Kd_ = declare_parameter<double>("rate_Kd", 0.002);
    rate_Kp_yaw_ = declare_parameter<double>("rate_Kp_yaw", 4.0);
    rate_Ki_yaw_ = declare_parameter<double>("rate_Ki_yaw", 0.05);
    rate_Kd_yaw_ = declare_parameter<double>("rate_Kd_yaw", 0.001);
    rate_integral_limit_ = declare_parameter<double>("rate_integral_limit", 1.5);
    rate_integral_leak_rate_ = declare_parameter<double>("rate_integral_leak_rate", 0.5);

    // initialize rate PID controllers
    rate_pid_roll_.set_gains(rate_Kp_, rate_Ki_, rate_Kd_);
    rate_pid_pitch_.set_gains(rate_Kp_, rate_Ki_, rate_Kd_);
    rate_pid_yaw_.set_gains(rate_Kp_yaw_, rate_Ki_yaw_, rate_Kd_yaw_);
    rate_pid_roll_.set_integral_limits(rate_integral_limit_, rate_integral_leak_rate_);
    rate_pid_pitch_.set_integral_limits(rate_integral_limit_, rate_integral_leak_rate_);
    rate_pid_yaw_.set_integral_limits(rate_integral_limit_, rate_integral_leak_rate_);
    altitude_pid_.set_gains(altitude_Kp_, altitude_Ki_, altitude_Kd_);
    altitude_pid_.set_integral_limits(altitude_integral_limit_, altitude_integral_leak_rate_);

    control_mode_ = ControlMode::MANUAL_ATTITUDE;
    position_target_initialized_ = false;

    // Allow runtime tuning: update gains when relevant parameters change
    on_set_parameters_callback_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) -> rcl_interfaces::msg::SetParametersResult {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & p : params) {
          const auto & name = p.get_name();
          if (name == "attitude_inertia") inertia_ = p.as_double();
          else if (name == "torque_to_speed_gain") torque_to_speed_gain_ = p.as_double();
          else if (name == "max_attitude_deg") max_attitude_deg_ = p.as_double();
          else if (name == "yaw_gain") yaw_gain_ = p.as_double();
          else if (name == "throttle_trim_range") throttle_trim_range_ = p.as_double();
          else if (name == "throttle_deadzone") throttle_deadzone_ = p.as_double();
          else if (name == "max_yaw_rate_deg") max_yaw_rate_deg_ = p.as_double();
          else if (name == "position_target_step_xy_m") position_target_step_xy_m_ = p.as_double();
          else if (name == "position_target_step_z_m") position_target_step_z_m_ = p.as_double();
          else if (name == "position_target_step_yaw_deg") position_target_step_yaw_deg_ = p.as_double();
          else if (name == "position_xy_gain") position_xy_gain_ = p.as_double();
          else if (name == "max_position_tilt_deg") max_position_tilt_deg_ = p.as_double();
          else if (name == "altitude_Kp") altitude_Kp_ = p.as_double();
          else if (name == "altitude_Ki") altitude_Ki_ = p.as_double();
          else if (name == "altitude_Kd") altitude_Kd_ = p.as_double();
          else if (name == "altitude_integral_limit") altitude_integral_limit_ = p.as_double();
          else if (name == "altitude_integral_leak_rate") altitude_integral_leak_rate_ = p.as_double();
          if (name == "K_R_attitude") K_R_attitude_ = p.as_double();
          else if (name == "K_R_yaw") K_R_yaw_ = p.as_double();
          else if (name == "attitude_inertia_z") attitude_inertia_z_ = p.as_double();
          else if (name == "rate_Kp") rate_Kp_ = p.as_double();
          else if (name == "rate_Ki") rate_Ki_ = p.as_double();
          else if (name == "rate_Kd") rate_Kd_ = p.as_double();
          else if (name == "rate_Kp_yaw") rate_Kp_yaw_ = p.as_double();
          else if (name == "rate_Ki_yaw") rate_Ki_yaw_ = p.as_double();
          else if (name == "rate_Kd_yaw") rate_Kd_yaw_ = p.as_double();
          else if (name == "rate_integral_limit") rate_integral_limit_ = p.as_double();
          else if (name == "rate_integral_leak_rate") rate_integral_leak_rate_ = p.as_double();
        }
        // apply gains to PID objects in case they were updated
        rate_pid_roll_.set_gains(rate_Kp_, rate_Ki_, rate_Kd_);
        rate_pid_pitch_.set_gains(rate_Kp_, rate_Ki_, rate_Kd_);
        rate_pid_yaw_.set_gains(rate_Kp_yaw_, rate_Ki_yaw_, rate_Kd_yaw_);
        rate_pid_roll_.set_integral_limits(rate_integral_limit_, rate_integral_leak_rate_);
        rate_pid_pitch_.set_integral_limits(rate_integral_limit_, rate_integral_leak_rate_);
        rate_pid_yaw_.set_integral_limits(rate_integral_limit_, rate_integral_leak_rate_);
        altitude_pid_.set_gains(altitude_Kp_, altitude_Ki_, altitude_Kd_);
        altitude_pid_.set_integral_limits(altitude_integral_limit_, altitude_integral_leak_rate_);
        rate_pid_roll_.reset();
        rate_pid_pitch_.reset();
        rate_pid_yaw_.reset();
        altitude_pid_.reset();
        return result;
      });

    const double timer_period_sec = publish_rate_hz_ > 0.0 ? (1.0 / publish_rate_hz_) : 0.02;
    control_dt_ = timer_period_sec;

    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, 10,
      std::bind(&DroneController::imu_callback, this, std::placeholders::_1));

    joy_subscription_ = create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_, 10,
      std::bind(&DroneController::joy_callback, this, std::placeholders::_1));

    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10,
      std::bind(&DroneController::odom_callback, this, std::placeholders::_1));

    motor_publisher_ = create_publisher<actuator_msgs::msg::Actuators>(motor_topic_, 10);
    position_target_publisher_ = create_publisher<nav_msgs::msg::Odometry>(position_target_topic_, 10);
    control_mode_publisher_ = create_publisher<std_msgs::msg::String>(control_mode_topic_, 10);
    position_target_service_ = create_service<drone_control::srv::SetPositionTarget>(
      "set_position_target",
      std::bind(
        &DroneController::handle_set_position_target_service,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
    control_mode_service_ = create_service<drone_control::srv::SetControlMode>(
      "set_control_mode",
      std::bind(
        &DroneController::handle_set_control_mode_service,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
    timer_ = create_wall_timer(
      std::chrono::duration<double>(timer_period_sec),
      std::bind(&DroneController::publish_command, this));

    RCLCPP_INFO(
      get_logger(),
        "Controller ready: imu=%s odom=%s joy=%s motor=%s motors=%d mode=%s",
        imu_topic_.c_str(), odom_topic_.c_str(), joy_topic_.c_str(), motor_topic_.c_str(), motor_count_,
        control_mode_name(control_mode_).c_str());
  }

  void stop()
  {
    publish_zero_command("shutdown");
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    last_imu_[0] = msg->orientation.x;
    last_imu_[1] = msg->orientation.y;
    last_imu_[2] = msg->orientation.z;
    last_imu_[3] = msg->orientation.w;
    // store angular velocity for rate feedback
    last_gyro_[0] = msg->angular_velocity.x;
    last_gyro_[1] = msg->angular_velocity.y;
    last_gyro_[2] = msg->angular_velocity.z;
    have_imu_ = true;
  }

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    previous_joy_buttons_ = last_joy_buttons_;
    last_joy_axes_.assign(msg->axes.begin(), msg->axes.end());
    last_joy_buttons_.assign(msg->buttons.begin(), msg->buttons.end());
    last_joy_time_ = now();
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_odom_ = *msg;
    have_odom_ = true;

    if (control_mode_ == ControlMode::POSITION_HOLD && !position_target_initialized_) {
      initialize_position_target_from_current_state();
    }
  }

  double joy_axis(int index) const
  {
    if (index < 0 || index >= static_cast<int>(last_joy_axes_.size())) {
      return 0.0;
    }
    return static_cast<double>(last_joy_axes_[static_cast<std::size_t>(index)]);
  }

  int joy_button(int index) const
  {
    if (index < 0 || index >= static_cast<int>(last_joy_buttons_.size())) {
      return 0;
    }
    return last_joy_buttons_[static_cast<std::size_t>(index)];
  }

  bool button_rising_edge(int index) const
  {
    if (index < 0 || index >= static_cast<int>(last_joy_buttons_.size())) {
      return false;
    }
    if (index >= static_cast<int>(previous_joy_buttons_.size())) {
      return false;
    }
    return last_joy_buttons_[static_cast<std::size_t>(index)] != 0 &&
      previous_joy_buttons_[static_cast<std::size_t>(index)] == 0;
  }

  static double wrap_angle(double angle)
  {
    while (angle > kPi) {
      angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
      angle += 2.0 * kPi;
    }
    return angle;
  }

  std::string control_mode_name(ControlMode mode) const
  {
    switch (mode) {
      case ControlMode::MANUAL_ATTITUDE:
        return "manual_attitude";
      case ControlMode::POSITION_HOLD:
        return "position_hold";
    }
    return "unknown";
  }

  void publish_control_mode_state()
  {
    std_msgs::msg::String mode_message;
    mode_message.data = control_mode_name(control_mode_);
    control_mode_publisher_->publish(mode_message);
  }

  void initialize_position_target_from_current_state()
  {
    if (!have_odom_ || !have_imu_) {
      position_target_initialized_ = false;
      return;
    }

    const auto euler = drone_control::quaternion_to_euler(
      last_imu_[0], last_imu_[1], last_imu_[2], last_imu_[3]);

    position_target_[0] = last_odom_.pose.pose.position.x;
    position_target_[1] = last_odom_.pose.pose.position.y;
    position_target_[2] = last_odom_.pose.pose.position.z;
    position_target_yaw_rad_ = euler[2];
    position_target_initialized_ = true;
  }

  double current_throttle_trim() const
  {
    double throttle_command = joy_axis(axis_throttle_);
    if (std::fabs(throttle_command) < throttle_deadzone_) {
      throttle_command = 0.0;
    }
    return drone_control::clamp_value(throttle_command, -1.0, 1.0) * throttle_trim_range_;
  }

  void begin_position_hold_mode()
  {
    position_target_initialized_ = false;
    position_hold_throttle_bias_ = current_throttle_trim();
    if (altitude_Ki_ > 0.0) {
      altitude_pid_.set_integral(position_hold_throttle_bias_ / altitude_Ki_);
    }
    if (have_odom_ && have_imu_) {
      initialize_position_target_from_current_state();
    }
  }

  void update_position_targets_from_joystick(double dt)
  {
    if (control_mode_ != ControlMode::POSITION_HOLD) {
      return;
    }

    position_target_[0] += joy_axis(axis_pitch_) * position_target_step_xy_m_ * dt;
    position_target_[1] += joy_axis(axis_roll_) * position_target_step_xy_m_ * dt;
    position_target_yaw_rad_ = wrap_angle(
      position_target_yaw_rad_ +
      joy_axis(axis_yaw_) * (position_target_step_yaw_deg_ * kPi / 180.0) * dt);
  }

  void handle_set_control_mode_service(
    const std::shared_ptr<drone_control::srv::SetControlMode::Request> request,
    std::shared_ptr<drone_control::srv::SetControlMode::Response> response)
  {
    const auto mode = static_cast<ControlMode>(request->mode);
    if (request->mode != static_cast<std::uint8_t>(ControlMode::MANUAL_ATTITUDE) &&
      request->mode != static_cast<std::uint8_t>(ControlMode::POSITION_HOLD))
    {
      response->success = false;
      response->message = "invalid control mode";
      return;
    }

    control_mode_ = mode;
    if (control_mode_ == ControlMode::POSITION_HOLD) {
      begin_position_hold_mode();
    }
    publish_control_mode_state();

    response->success = true;
    response->message = "control mode set to " + control_mode_name(control_mode_);
  }

  void handle_set_position_target_service(
    const std::shared_ptr<drone_control::srv::SetPositionTarget::Request> request,
    std::shared_ptr<drone_control::srv::SetPositionTarget::Response> response)
  {
    position_target_[0] = request->x;
    position_target_[1] = request->y;
    position_target_[2] = request->z;
    position_target_yaw_rad_ = wrap_angle(request->yaw_deg * kPi / 180.0);
    position_target_initialized_ = true;

    response->success = true;
    response->message = "position target set";
    RCLCPP_INFO(
      get_logger(),
      "Position target set to x=%.3f y=%.3f z=%.3f yaw=%.1f deg",
      position_target_[0], position_target_[1], position_target_[2], request->yaw_deg);
  }

  void update_control_mode_from_joystick()
  {
    if (button_rising_edge(mode_toggle_button_)) {
      if (control_mode_ == ControlMode::MANUAL_ATTITUDE) {
        control_mode_ = ControlMode::POSITION_HOLD;
        begin_position_hold_mode();
      } else {
        control_mode_ = ControlMode::MANUAL_ATTITUDE;
      }
      publish_control_mode_state();
      RCLCPP_INFO(get_logger(), "Control mode changed to %s", control_mode_name(control_mode_).c_str());
    }

    if (control_mode_ == ControlMode::POSITION_HOLD && button_rising_edge(mode_reset_button_)) {
      initialize_position_target_from_current_state();
      RCLCPP_INFO(get_logger(), "Position target reset to current state");
    }
  }

  void publish_zero_command(const char * reason)
  {
    RCLCPP_DEBUG(get_logger(), "Zeroing motors: %s", reason);
    actuator_msgs::msg::Actuators command;
    command.velocity.assign(static_cast<std::size_t>(std::max(motor_count_, 0)), min_motor_speed_);
    motor_publisher_->publish(command);
  }

  void publish_position_target_command()
  {
    if (!position_target_initialized_) {
      return;
    }

    nav_msgs::msg::Odometry target;
    target.header.stamp = now();
    target.header.frame_id = odom_topic_;
    target.child_frame_id = "position_target";
    target.pose.pose.position.x = position_target_[0];
    target.pose.pose.position.y = position_target_[1];
    target.pose.pose.position.z = position_target_[2];

    const double half_yaw = 0.5 * position_target_yaw_rad_;
    target.pose.pose.orientation.x = 0.0;
    target.pose.pose.orientation.y = 0.0;
    target.pose.pose.orientation.z = std::sin(half_yaw);
    target.pose.pose.orientation.w = std::cos(half_yaw);

    position_target_publisher_->publish(target);
  }

  void publish_command()
  {
    update_control_mode_from_joystick();
    publish_control_mode_state();

    const rclcpp::Duration joy_timeout = rclcpp::Duration::from_seconds(joy_timeout_sec_);
    if ((now() - last_joy_time_) > joy_timeout) {
      publish_zero_command("joy timeout");
      return;
    }

    if (joy_button(deadman_button_) == 0) {
      publish_zero_command("deadman not pressed");
      return;
    }

    const double roll_axis = joy_axis(axis_roll_);
    const double pitch_axis = joy_axis(axis_pitch_);
    const double yaw_axis = joy_axis(axis_yaw_);

    const double throttle_trim = current_throttle_trim();
    double base_speed = hover_motor_speed_ + throttle_trim;
    base_speed = drone_control::clamp_value(base_speed, min_motor_speed_, max_motor_speed_);

    double desired_roll = roll_axis * (max_attitude_deg_ * kPi / 180.0);
    double desired_pitch = pitch_axis * (max_attitude_deg_ * kPi / 180.0);
    double desired_yaw_rate = yaw_axis * (max_yaw_rate_deg_ * kPi / 180.0);

    if (control_mode_ == ControlMode::POSITION_HOLD) {
      if (!have_odom_ || !have_imu_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "position hold waiting for odom/imu; using hover fallback");
        base_speed = drone_control::clamp_value(
          hover_motor_speed_ + position_hold_throttle_bias_,
          min_motor_speed_,
          max_motor_speed_);
      }

      if (have_odom_ && have_imu_ && !position_target_initialized_) {
        initialize_position_target_from_current_state();
      }

      if (have_odom_ && have_imu_) {
        update_position_targets_from_joystick(control_dt_);

        const auto euler = drone_control::quaternion_to_euler(
          last_imu_[0], last_imu_[1], last_imu_[2], last_imu_[3]);
        const double current_yaw = euler[2];
        const double current_x = last_odom_.pose.pose.position.x;
        const double current_y = last_odom_.pose.pose.position.y;
        const double current_z = last_odom_.pose.pose.position.z;

        const double error_x = position_target_[0] - current_x;
        const double error_y = position_target_[1] - current_y;
        const double error_z = position_target_[2] - current_z;

        const double cy = std::cos(current_yaw);
        const double sy = std::sin(current_yaw);
        const double forward_error = cy * error_x + sy * error_y;
        const double right_error = -sy * error_x + cy * error_y;

        const double max_position_tilt_rad = max_position_tilt_deg_ * kPi / 180.0;
        desired_pitch = drone_control::clamp_value(
          position_xy_gain_ * forward_error,
          -max_position_tilt_rad,
          max_position_tilt_rad);
        desired_roll = drone_control::clamp_value(
          -position_xy_gain_ * right_error,
          -max_position_tilt_rad,
          max_position_tilt_rad);

        const double throttle_trim_position = drone_control::clamp_value(
          altitude_pid_.update(error_z, control_dt_),
          min_motor_speed_ - hover_motor_speed_,
          max_motor_speed_ - hover_motor_speed_);
        base_speed = drone_control::clamp_value(
          hover_motor_speed_ + throttle_trim_position,
          min_motor_speed_,
          max_motor_speed_);

        const double yaw_error = wrap_angle(position_target_yaw_rad_ - current_yaw);
        desired_yaw_rate = drone_control::clamp_value(
          K_R_yaw_ * yaw_error,
          -max_yaw_rate_deg_ * kPi / 180.0,
          max_yaw_rate_deg_ * kPi / 180.0);

        publish_position_target_command();
      }
    }

    // Attitude control (roll and pitch)
    double roll_delta = 0.0;
    double pitch_delta = 0.0;
    double yaw_term = yaw_axis * yaw_gain_;
    if (have_imu_) {
      const auto euler = drone_control::quaternion_to_euler(
        last_imu_[0], last_imu_[1], last_imu_[2], last_imu_[3]);

      const double roll_rate = last_gyro_[0];
      const double pitch_rate = last_gyro_[1];

      // Geometric (SO(3)) attitude control using utils
      auto R = drone_control::R_from_quat(last_imu_);
      const double current_yaw = euler[2];
      // Keep the attitude reference aligned with the current yaw frame.
      // Yaw is controlled separately as a rate, so it should not distort roll/pitch correction.
      auto Rd = drone_control::mat_mul(drone_control::Rz(current_yaw),
                   drone_control::mat_mul(drone_control::Ry(desired_pitch),
                          drone_control::Rx(desired_roll)));

      auto RdT = drone_control::mat_transpose(Rd);
      auto RT = drone_control::mat_transpose(R);
      auto temp1 = drone_control::mat_mul(RdT, R);
      auto temp2 = drone_control::mat_mul(RT, Rd);
      std::array<std::array<double,3>,3> diff{};
      for (int i=0;i<3;++i) for (int j=0;j<3;++j) diff[i][j] = temp1[i][j] - temp2[i][j];
      auto eR = drone_control::vee(diff);
      eR[0] *= 0.5; eR[1] *= 0.5; eR[2] *= 0.5;

      // measured angular velocity
      std::array<double,3> omega = {roll_rate, pitch_rate, last_gyro_[2]};

      // Outer-loop: desired body rates from attitude error (roll/pitch)
      std::array<double,3> omega_des{};
      omega_des[0] = K_R_attitude_ * eR[0];
      omega_des[1] = K_R_attitude_ * eR[1];
      omega_des[2] = desired_yaw_rate;

      // Inertia and Coriolis-like cross term
      double Ixx = inertia_;
      double Iyy = inertia_;
      double Izz = attitude_inertia_z_;
      std::array<double,3> cross{
        omega[1]*omega[2]*(Iyy - Izz),
        omega[2]*omega[0]*(Izz - Ixx),
        omega[0]*omega[1]*(Ixx - Iyy)
      };

      // Inner-loop: rate PID -> moments (use RatePID objects)
      std::array<double,3> rate_err{};
      for (int i=0;i<3;++i) rate_err[i] = omega[i] - omega_des[i];

      double Mx = -rate_pid_roll_.update(rate_err[0], control_dt_) + cross[0];
      double My = -rate_pid_pitch_.update(rate_err[1], control_dt_) + cross[1];
      double Mz = -rate_pid_yaw_.update(rate_err[2], control_dt_) + cross[2];

      roll_delta = Mx * torque_to_speed_gain_;
      pitch_delta = My * torque_to_speed_gain_;
      yaw_term = Mz * torque_to_speed_gain_;
    }

    std::vector<double> motor_speeds;
    if (motor_count_ != 4) {
      if (!warned_non_quad_) {
        RCLCPP_WARN(
          get_logger(),
          "motor_count=%d is not 4, publishing base speed to all outputs",
          motor_count_);
        warned_non_quad_ = true;
      }
      motor_speeds.assign(static_cast<std::size_t>(std::max(motor_count_, 0)), base_speed);
    } else {
      auto mixed = drone_control::quad_mix(base_speed, roll_delta, pitch_delta, yaw_term);
      motor_speeds.resize(4);
      for (size_t i=0;i<4;++i) {
        motor_speeds[i] = drone_control::clamp_value(mixed[i], min_motor_speed_, max_motor_speed_);
      }
    }

    actuator_msgs::msg::Actuators command;
    command.velocity = motor_speeds;
    motor_publisher_->publish(command);
  }

  std::string imu_topic_;
  std::string odom_topic_;
  std::string joy_topic_;
  std::string motor_topic_;
  std::string position_target_topic_;
  std::string control_mode_topic_;

  double publish_rate_hz_ {50.0};
  int motor_count_ {4};
  double min_motor_speed_ {0.0};
  double max_motor_speed_ {1000.0};
  double hover_motor_speed_ {505.0};
  int deadman_button_ {4};
  double joy_timeout_sec_ {0.5};
  int axis_roll_ {0};
  int axis_pitch_ {1};
  int axis_yaw_ {2};
  int axis_throttle_ {3};
  int mode_toggle_button_ {6};
  int mode_reset_button_ {7};
  // IMU state
  std::array<double, 4> last_imu_ {};
  std::array<double, 3> last_gyro_ {};
  bool have_imu_ {false};
  nav_msgs::msg::Odometry last_odom_ {};
  bool have_odom_ {false};
  // inertia
  double inertia_ {0.02};
  double torque_to_speed_gain_ {0.5};
  double max_attitude_deg_ {30.0};
  double yaw_gain_ {50.0};
  double throttle_trim_range_ {80.0};
  double throttle_deadzone_ {0.08};
  // Geometric controller gains
  double K_R_attitude_ {8.0};
  double K_R_yaw_ {4.0};
  double attitude_inertia_z_ {0.02};
  // Rate PID inner-loop gains and state
  double rate_Kp_ {6.0};
  double rate_Ki_ {0.1};
  double rate_Kd_ {0.002};
  double rate_Kp_yaw_ {4.0};
  double rate_Ki_yaw_ {0.05};
  double rate_Kd_yaw_ {0.001};
  double rate_integral_limit_ {1.5};
  double rate_integral_leak_rate_ {0.5};
  double max_yaw_rate_deg_ {90.0};
  double position_target_step_xy_m_ {0.10};
  double position_target_step_z_m_ {0.05};
  double position_target_step_yaw_deg_ {10.0};
  double position_xy_gain_ {0.35};
  double max_position_tilt_deg_ {15.0};
  double altitude_Kp_ {120.0};
  double altitude_Ki_ {0.0};
  double altitude_Kd_ {0.0};
  double altitude_integral_limit_ {200.0};
  double altitude_integral_leak_rate_ {0.5};
  double position_hold_throttle_bias_ {0.0};
  drone_control::RatePID rate_pid_roll_;
  drone_control::RatePID rate_pid_pitch_;
  drone_control::RatePID rate_pid_yaw_;
  drone_control::RatePID altitude_pid_;
  double control_dt_ {0.02};
  ControlMode control_mode_ {ControlMode::MANUAL_ATTITUDE};
  std::array<double, 3> position_target_ {0.0, 0.0, 0.0};
  double position_target_yaw_rad_ {0.0};
  bool position_target_initialized_ {false};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_ {nullptr};
  std::vector<float> last_joy_axes_;
  std::vector<std::int32_t> previous_joy_buttons_;
  std::vector<std::int32_t> last_joy_buttons_;
  rclcpp::Time last_joy_time_ {0, 0, RCL_ROS_TIME};
  bool warned_non_quad_ {false};

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Service<drone_control::srv::SetPositionTarget>::SharedPtr position_target_service_;
  rclcpp::Service<drone_control::srv::SetControlMode>::SharedPtr control_mode_service_;
  rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr motor_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr position_target_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_mode_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DroneController>();
  rclcpp::spin(node);
  node->stop();
  rclcpp::shutdown();
  return 0;
}