#include "world/heading_lock_node.hpp"

#include <algorithm>
#include <sstream>

namespace world {

double HeadingLockNode::wrap_pi(double a) {
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a < 0.0) a += 2.0 * M_PI;
  return a - M_PI;
}

HeadingLockNode::HeadingLockNode() : Node("heading_lock_node") {
  declare_parameter("rate_hz", 100.0);
  declare_parameter("kp", 2.0);
  declare_parameter("ki", 0.05);
  declare_parameter("kd", 0.1);
  declare_parameter("i_clamp", 0.5);
  declare_parameter("max_omega", 1.5);
  declare_parameter("deadband_deg", 1.0);
  declare_parameter("deriv_alpha", 0.3);

  rate_hz_ = get_parameter("rate_hz").as_double();
  kp_ = get_parameter("kp").as_double();
  ki_ = get_parameter("ki").as_double();
  kd_ = get_parameter("kd").as_double();
  i_clamp_ = get_parameter("i_clamp").as_double();
  max_omega_ = get_parameter("max_omega").as_double();
  deadband_rad_ = get_parameter("deadband_deg").as_double() * M_PI / 180.0;
  deriv_alpha_ = get_parameter("deriv_alpha").as_double();
  dt_ = 1.0 / rate_hz_;

  param_cb_handle_ = add_on_set_parameters_callback(
      std::bind(&HeadingLockNode::on_params, this, std::placeholders::_1));

  sub_target_ = create_subscription<std_msgs::msg::Float32>(
      "/target_heading", 10,
      std::bind(&HeadingLockNode::target_cb, this, std::placeholders::_1));
  sub_imu_ = create_subscription<std_msgs::msg::Float32>(
      "/imu_yaw", 10,
      std::bind(&HeadingLockNode::imu_cb, this, std::placeholders::_1));

  pub_omega_ = create_publisher<std_msgs::msg::Float32>("/heading_omega", 10);
  pub_status_ = create_publisher<std_msgs::msg::String>("/heading_lock/status", 5);

  timer_pid_ = create_wall_timer(
      std::chrono::duration<double>(dt_),
      std::bind(&HeadingLockNode::pid_loop, this));
  timer_diag_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&HeadingLockNode::pub_diag, this));

  RCLCPP_INFO(get_logger(), "HeadingLockNode started at %.0f Hz", rate_hz_);
}

rcl_interfaces::msg::SetParametersResult
HeadingLockNode::on_params(const std::vector<rclcpp::Parameter> &params) {
  for (const auto &p : params) {
    const auto &n = p.get_name();
    if      (n == "kp")           kp_ = p.as_double();
    else if (n == "ki")           ki_ = p.as_double();
    else if (n == "kd")           kd_ = p.as_double();
    else if (n == "i_clamp")      i_clamp_ = p.as_double();
    else if (n == "max_omega")    max_omega_ = p.as_double();
    else if (n == "deadband_deg") deadband_rad_ = p.as_double() * M_PI / 180.0;
    else if (n == "deriv_alpha")  deriv_alpha_ = p.as_double();
  }
  rcl_interfaces::msg::SetParametersResult res;
  res.successful = true;
  return res;
}

void HeadingLockNode::target_cb(const std_msgs::msg::Float32::SharedPtr msg) {
  const double new_rad = msg->data * M_PI / 180.0;

  // D-kick prevention: reset PID when target changes significantly
  if (!prev_target_rad_.has_value() ||
      std::abs(wrap_pi(new_rad - prev_target_rad_.value())) > deadband_rad_) {
    pid_reset_ = true;
  }

  target_rad_ = new_rad;
  prev_target_rad_ = new_rad;
  have_target_ = true;
}

void HeadingLockNode::imu_cb(const std_msgs::msg::Float32::SharedPtr msg) {
  imu_yaw_rad_ = msg->data * M_PI / 180.0;
}

void HeadingLockNode::pid_loop() {
  std_msgs::msg::Float32 out;

  if (!have_target_) {
    out.data = 0.0f;
    pub_omega_->publish(out);
    return;
  }

  const double error = wrap_pi(target_rad_ - imu_yaw_rad_);

  // Deadband — settle cleanly
  if (std::abs(error) < deadband_rad_) {
    integral_ = 0.0;
    pid_reset_ = true;
    last_error_rad_ = error;
    last_output_ = 0.0;
    out.data = 0.0f;
    pub_omega_->publish(out);
    return;
  }

  // D-kick prevention on reset
  if (pid_reset_) {
    prev_error_ = error;
    filt_deriv_ = 0.0;
    integral_ = 0.0;
    pid_reset_ = false;
  }

  // Filtered derivative
  const double raw_deriv = (error - prev_error_) / dt_;
  filt_deriv_ = deriv_alpha_ * raw_deriv + (1.0 - deriv_alpha_) * filt_deriv_;
  prev_error_ = error;

  // Integral with anti-windup
  integral_ += error * dt_;
  if (ki_ > 1e-9) {
    const double i_max = i_clamp_ / ki_;
    integral_ = std::clamp(integral_, -i_max, i_max);
  }

  // PID output
  double output = kp_ * error + ki_ * integral_ + kd_ * filt_deriv_;
  output = std::clamp(output, -max_omega_, max_omega_);

  last_error_rad_ = error;
  last_output_ = output;

  out.data = static_cast<float>(output);
  pub_omega_->publish(out);
}

void HeadingLockNode::pub_diag() {
  std::ostringstream ss;
  ss << "{\"have_target\":" << (have_target_ ? "true" : "false")
     << ",\"error_deg\":" << (last_error_rad_ * 180.0 / M_PI)
     << ",\"output\":" << last_output_
     << ",\"integral\":" << integral_ << "}";

  std_msgs::msg::String msg;
  msg.data = ss.str();
  pub_status_->publish(msg);
}

}  // namespace world

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<world::HeadingLockNode>());
  rclcpp::shutdown();
  return 0;
}
