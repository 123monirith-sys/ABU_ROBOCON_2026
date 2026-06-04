#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <array>
#include <atomic>
#include <cmath>
#include <vector>

namespace world {

class SwerveControllerNode : public rclcpp::Node {
public:
  SwerveControllerNode();

private:
  void field_cmd_cb(const geometry_msgs::msg::Twist::SharedPtr msg);
  void imu_cb(const std_msgs::msg::Float32::SharedPtr msg);
  void heading_omega_cb(const std_msgs::msg::Float32::SharedPtr msg);
  void control_loop();
  void pub_diag();

  // Field→body rotation (variant C: correct inverse of odom_node body→world)
  void field_to_body(double vx_f, double vy_f, double yaw,
                     double &vx_r, double &vy_r) const;

  // Desaturate so no wheel exceeds max_speed_ms
  void desaturate(double &vx, double &vy, double &omega) const;

  static double ramp(double cur, double target, double max_step);

  // Live parameter update (enables runtime tuning via the parameter service)
  rcl_interfaces::msg::SetParametersResult
  on_params(const std::vector<rclcpp::Parameter> &params);

  // Parameters
  double rate_hz_, dt_;
  bool field_centric_;
  double deadband_xy_, deadband_omega_;
  double slew_xy_, slew_omega_;
  double max_speed_ms_;
  double wheel_radius_, robot_l_, robot_w_;
  double heading_omega_weight_;
  double cmd_timeout_s_;

  // Wheel positions [FL, FR, BL, BR]
  std::array<double, 4> lx_, ly_;

  // Input state
  double vx_field_ = 0.0, vy_field_ = 0.0;
  double omega_cmd_ = 0.0;
  double imu_yaw_rad_ = 0.0;
  double heading_omega_ = 0.0;
  rclcpp::Time last_cmd_time_;

  // Slew-rate state (robot frame)
  double cur_vx_ = 0.0, cur_vy_ = 0.0, cur_omega_ = 0.0;

  // ROS I/O
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_field_cmd_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_imu_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_heading_omega_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_climb_active_;
  std::atomic<bool> climb_active_{false};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
  rclcpp::TimerBase::SharedPtr timer_control_;
  rclcpp::TimerBase::SharedPtr timer_diag_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

}  // namespace world
