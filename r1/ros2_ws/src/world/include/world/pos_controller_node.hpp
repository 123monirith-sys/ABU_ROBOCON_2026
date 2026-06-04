#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <cmath>
#include <optional>
#include <tuple>
#include <vector>

namespace world {

class PosControllerNode : public rclcpp::Node {
public:
  PosControllerNode();

private:
  static double wrap_pi(double a);

  void target_cb(const geometry_msgs::msg::Pose2D::SharedPtr msg);
  void pose_cb(const geometry_msgs::msg::Pose2D::SharedPtr msg);
  void loop();
  void pub_zero_cmd(bool arrived);
  void pub_diag();

  // Live parameter update (enables runtime tuning via the parameter service)
  rcl_interfaces::msg::SetParametersResult
  on_params(const std::vector<rclcpp::Parameter> &params);

  // Parameters
  double rate_hz_, dt_;
  double kp_xy_, kd_xy_;
  double kp_xy_near_, kd_xy_near_;
  double near_thresh_;
  double kp_theta_, kd_theta_;
  double dead_xy_, dead_theta_;       // radians for theta
  double hold_xy_, hold_theta_;       // radians for theta
  double max_v_, max_w_;
  double alpha_;

  // Current pose
  double X_ = 0.0, Y_ = 0.0, theta_ = 0.0;
  bool have_pose_ = false;

  // Target pose
  double x_ref_ = 0.0, y_ref_ = 0.0, theta_ref_ = 0.0;
  bool have_target_ = false;
  std::optional<std::tuple<double, double, double>> prev_target_;

  // PD state
  double prev_ex_ = 0.0, prev_ey_ = 0.0, prev_et_ = 0.0;
  double filt_dex_ = 0.0, filt_dey_ = 0.0, filt_det_ = 0.0;
  bool pd_reset_ = true;
  bool arrived_ = false;

  // Diagnostic cache
  double last_ex_ = 0.0, last_ey_ = 0.0, last_et_ = 0.0, last_dist_ = 0.0;
  double last_vx_ = 0.0, last_vy_ = 0.0, last_omega_ = 0.0;

  // ROS I/O
  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr sub_target_;
  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr sub_pose_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_arrived_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
  rclcpp::TimerBase::SharedPtr timer_loop_;
  rclcpp::TimerBase::SharedPtr timer_diag_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

}  // namespace world
