#include "world/odom_node.hpp"

namespace world {

OdomNode::OdomNode() : Node("odom_node") {
  declare_parameter("rate_hz", 250.0);
  rate_hz_ = get_parameter("rate_hz").as_double();
  dt_ = 1.0 / rate_hz_;

  sub_ekf_ = create_subscription<geometry_msgs::msg::Pose2D>(
      "/ekf/pose", 10,
      std::bind(&OdomNode::ekf_cb, this, std::placeholders::_1));
  sub_imu_ = create_subscription<std_msgs::msg::Float32>(
      "/imu_yaw", 10,
      std::bind(&OdomNode::imu_cb, this, std::placeholders::_1));
  sub_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&OdomNode::cmd_cb, this, std::placeholders::_1));

  pub_ = create_publisher<geometry_msgs::msg::Pose2D>("/odom_fast", 10);

  timer_ = create_wall_timer(
      std::chrono::duration<double>(dt_),
      std::bind(&OdomNode::integrate, this));

  RCLCPP_INFO(get_logger(), "OdomNode started at %.0f Hz (anchored to /ekf/pose)",
              rate_hz_);
}

void OdomNode::ekf_cb(const geometry_msgs::msg::Pose2D::SharedPtr msg) {
  x_ = msg->x;
  y_ = msg->y;
}

void OdomNode::imu_cb(const std_msgs::msg::Float32::SharedPtr msg) {
  imu_yaw_rad_ = msg->data * M_PI / 180.0;
}

void OdomNode::cmd_cb(const geometry_msgs::msg::Twist::SharedPtr msg) {
  vx_robot_ = msg->linear.x;  // strafe
  vy_robot_ = msg->linear.y;  // forward
}

void OdomNode::integrate() {
  // Body→world rotation (body x=strafe, y=forward)
  // world_x aligns with robot strafe at yaw=0, world_y with forward
  const double cos_th = std::cos(imu_yaw_rad_);
  const double sin_th = std::sin(imu_yaw_rad_);

  const double vx_world = cos_th * vx_robot_ - sin_th * vy_robot_;
  const double vy_world = sin_th * vx_robot_ + cos_th * vy_robot_;

  x_ += vx_world * dt_;
  y_ += vy_world * dt_;

  geometry_msgs::msg::Pose2D pose;
  pose.x = x_;
  pose.y = y_;
  pose.theta = imu_yaw_rad_;
  pub_->publish(pose);
}

}  // namespace world

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<world::OdomNode>());
  rclcpp::shutdown();
  return 0;
}
