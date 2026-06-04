#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <array>
#include <cmath>
#include <optional>

namespace kinematic {

class EkfNode : public rclcpp::Node {
public:
  EkfNode();

private:
  static double wrap_angle(double a);

  void odom_cb(const geometry_msgs::msg::Pose2D::SharedPtr msg);
  void imu_cb(const std_msgs::msg::Float32::SharedPtr msg);
  void vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg);
  void set_pose_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void step();

  // Parameters
  double rate_hz_, dt_;
  double q_xy_, q_theta_;
  double r_imu_, r_enc_xy_;
  bool use_enc_update_;

  // EKF state [X, Y, theta]
  std::array<double, 3> x_;

  // Covariance 3x3
  std::array<std::array<double, 3>, 3> P_;
  std::array<std::array<double, 3>, 3> Q_;

  // Robot-frame velocity from /cmd_vel
  double vx_ = 0.0;       // strafe
  double vy_ = 0.0;       // forward
  double omega_z_ = 0.0;  // commanded yaw rate (rad/s) — used for midpoint integration

  // Measurements
  std::optional<double> imu_yaw_rad_;
  std::optional<double> enc_x_, enc_y_;
  std::optional<double> prev_enc_x_, prev_enc_y_;

  // ROS I/O
  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr sub_odom_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_imu_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_vel_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_set_pose_;
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pub_pose_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace kinematic
