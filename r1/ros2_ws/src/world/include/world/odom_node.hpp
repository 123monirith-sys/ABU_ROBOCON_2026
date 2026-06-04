#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cmath>

namespace world {

class OdomNode : public rclcpp::Node {
public:
  OdomNode();

private:
  void ekf_cb(const geometry_msgs::msg::Pose2D::SharedPtr msg);
  void imu_cb(const std_msgs::msg::Float32::SharedPtr msg);
  void cmd_cb(const geometry_msgs::msg::Twist::SharedPtr msg);
  void integrate();

  // Parameters
  double rate_hz_;
  double dt_;

  // Dead-reckoning state (world frame)
  double x_ = 0.0;
  double y_ = 0.0;
  double imu_yaw_rad_ = 0.0;

  // Robot-frame velocity from /cmd_vel
  double vx_robot_ = 0.0;  // strafe (linear.x)
  double vy_robot_ = 0.0;  // forward (linear.y)

  // ROS I/O
  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr sub_ekf_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_imu_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace world
