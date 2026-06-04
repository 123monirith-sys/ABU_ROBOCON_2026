#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <cmath>
#include <optional>
#include <vector>

namespace world {

class HeadingLockNode : public rclcpp::Node {
public:
  HeadingLockNode();

private:
  static double wrap_pi(double a);

  void target_cb(const std_msgs::msg::Float32::SharedPtr msg);
  void imu_cb(const std_msgs::msg::Float32::SharedPtr msg);
  void pid_loop();
  void pub_diag();

  // Live parameter update (enables runtime tuning via the parameter service)
  rcl_interfaces::msg::SetParametersResult
  on_params(const std::vector<rclcpp::Parameter> &params);

  // Parameters
  double rate_hz_, dt_;
  double kp_, ki_, kd_;
  double i_clamp_, max_omega_;
  double deadband_rad_, deriv_alpha_;

  // Input state
  double target_rad_ = 0.0;
  double imu_yaw_rad_ = 0.0;
  bool have_target_ = false;
  std::optional<double> prev_target_rad_;

  // PID state
  double integral_ = 0.0;
  double prev_error_ = 0.0;
  double filt_deriv_ = 0.0;
  bool pid_reset_ = true;

  // Diagnostic cache
  double last_error_rad_ = 0.0;
  double last_output_ = 0.0;

  // ROS I/O
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_target_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_imu_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_omega_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
  rclcpp::TimerBase::SharedPtr timer_pid_;
  rclcpp::TimerBase::SharedPtr timer_diag_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

}  // namespace world
