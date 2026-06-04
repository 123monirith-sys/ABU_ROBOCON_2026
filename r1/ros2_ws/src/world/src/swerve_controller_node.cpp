#include "world/swerve_controller_node.hpp"

#include <algorithm>
#include <sstream>

namespace world {

SwerveControllerNode::SwerveControllerNode() : Node("swerve_controller_node") {
  declare_parameter("rate_hz", 100.0);
  declare_parameter("field_centric", true);
  declare_parameter("deadband_xy", 0.02);
  declare_parameter("deadband_omega", 0.03);
  declare_parameter("slew_xy", 3.0);
  declare_parameter("slew_omega", 6.0);
  declare_parameter("max_speed_ms", 0.5);
  declare_parameter("wheel_radius", 0.039);
  declare_parameter("robot_l", 0.426);
  declare_parameter("robot_w", 0.426);
  declare_parameter("heading_omega_weight", 1.0);
  declare_parameter("cmd_timeout_s", 0.3);

  rate_hz_ = get_parameter("rate_hz").as_double();
  field_centric_ = get_parameter("field_centric").as_bool();
  deadband_xy_ = get_parameter("deadband_xy").as_double();
  deadband_omega_ = get_parameter("deadband_omega").as_double();
  slew_xy_ = get_parameter("slew_xy").as_double();
  slew_omega_ = get_parameter("slew_omega").as_double();
  max_speed_ms_ = get_parameter("max_speed_ms").as_double();
  wheel_radius_ = get_parameter("wheel_radius").as_double();
  robot_l_ = get_parameter("robot_l").as_double();
  robot_w_ = get_parameter("robot_w").as_double();
  heading_omega_weight_ = get_parameter("heading_omega_weight").as_double();
  cmd_timeout_s_ = get_parameter("cmd_timeout_s").as_double();
  dt_ = 1.0 / rate_hz_;
  last_cmd_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  // Wheel positions matching IK convention: FL, FR, BL, BR
  const double l2 = robot_l_ * 0.5;
  const double w2 = robot_w_ * 0.5;
  lx_ = {+l2, +l2, -l2, -l2};
  ly_ = {+w2, -w2, +w2, -w2};

  sub_field_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
      "/field_cmd_vel", 10,
      std::bind(&SwerveControllerNode::field_cmd_cb, this, std::placeholders::_1));
  sub_imu_ = create_subscription<std_msgs::msg::Float32>(
      "/imu_yaw", 10,
      std::bind(&SwerveControllerNode::imu_cb, this, std::placeholders::_1));
  sub_heading_omega_ = create_subscription<std_msgs::msg::Float32>(
      "/heading_omega", 10,
      std::bind(&SwerveControllerNode::heading_omega_cb, this, std::placeholders::_1));
  {
    rclcpp::QoS climb_qos(1);
    climb_qos.transient_local();
    sub_climb_active_ = create_subscription<std_msgs::msg::Bool>(
        "/climb/active", climb_qos,
        [this](const std_msgs::msg::Bool::SharedPtr m) {
          climb_active_.store(m->data);
        });
  }

  param_cb_handle_ = add_on_set_parameters_callback(
      std::bind(&SwerveControllerNode::on_params, this, std::placeholders::_1));

  pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  pub_status_ = create_publisher<std_msgs::msg::String>("/world/status", 5);

  timer_control_ = create_wall_timer(
      std::chrono::duration<double>(dt_),
      std::bind(&SwerveControllerNode::control_loop, this));
  timer_diag_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&SwerveControllerNode::pub_diag, this));

  RCLCPP_INFO(get_logger(),
              "SwerveControllerNode started at %.0f Hz "
              "(field_centric=%s, max_speed=%.2f m/s)",
              rate_hz_, field_centric_ ? "true" : "false", max_speed_ms_);
}

rcl_interfaces::msg::SetParametersResult
SwerveControllerNode::on_params(const std::vector<rclcpp::Parameter> &params) {
  for (const auto &p : params) {
    const auto &n = p.get_name();
    if      (n == "field_centric")        field_centric_ = p.as_bool();
    else if (n == "deadband_xy")          deadband_xy_ = p.as_double();
    else if (n == "deadband_omega")       deadband_omega_ = p.as_double();
    else if (n == "slew_xy")              slew_xy_ = p.as_double();
    else if (n == "slew_omega")           slew_omega_ = p.as_double();
    else if (n == "max_speed_ms")         max_speed_ms_ = p.as_double();
    else if (n == "heading_omega_weight") heading_omega_weight_ = p.as_double();
    else if (n == "cmd_timeout_s")        cmd_timeout_s_ = p.as_double();
  }
  rcl_interfaces::msg::SetParametersResult res;
  res.successful = true;
  return res;
}

void SwerveControllerNode::field_cmd_cb(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  vx_field_ = -msg->linear.x;
  vy_field_ = msg->linear.y;
  omega_cmd_ = msg->angular.z;
  last_cmd_time_ = now();
}

void SwerveControllerNode::imu_cb(
    const std_msgs::msg::Float32::SharedPtr msg) {
  imu_yaw_rad_ = msg->data * M_PI / 180.0;
}

void SwerveControllerNode::heading_omega_cb(
    const std_msgs::msg::Float32::SharedPtr msg) {
  heading_omega_ = msg->data;
}

void SwerveControllerNode::field_to_body(
    double vx_f, double vy_f, double yaw,
    double &vx_r, double &vy_r) const {
  // Inverse of odom body→world with world_x=strafe-axis, world_y=forward-axis
  // odom: world_x = cos*strafe - sin*fwd,  world_y = sin*strafe + cos*fwd
  // Inverse: strafe =  cos*world_x + sin*world_y
  //          fwd    = -sin*world_x + cos*world_y
  const double cos_y = std::cos(yaw);
  const double sin_y = std::sin(yaw);
  vx_r =  cos_y * vx_f - sin_y * vy_f;  // strafe (robot x)
  vy_r =  sin_y * vx_f + cos_y * vy_f;  // forward (robot y)
}

void SwerveControllerNode::desaturate(double &vx, double &vy,
                                       double &omega) const {
  // Mirror IK convention: vb_x = vy (forward), vb_y = vx (strafe)
  const double vb_x = vy;
  const double vb_y = vx;
  const double omega_ik = -omega;

  double max_spd = 0.0;
  for (int i = 0; i < 4; ++i) {
    const double wx = vb_x + omega_ik * ly_[i];
    const double wy = vb_y - omega_ik * lx_[i];
    const double spd = std::hypot(wx, wy);
    max_spd = std::max(max_spd, spd);
  }

  if (max_spd > max_speed_ms_) {
    const double scale = max_speed_ms_ / max_spd;
    vx *= scale;
    vy *= scale;
    omega *= scale;
  }
}

double SwerveControllerNode::ramp(double cur, double target,
                                   double max_step) {
  return cur + std::clamp(target - cur, -max_step, max_step);
}

void SwerveControllerNode::control_loop() {
  double vx_f = vx_field_;
  double vy_f = vy_field_;
  double omega = omega_cmd_;

  // Watchdog: if no /field_cmd_vel within cmd_timeout_s, treat as zero.
  // Protects against a publisher dying (e.g. Ctrl-C on `ros2 topic pub`)
  // leaving the last command latched forever.
  if (last_cmd_time_.nanoseconds() == 0 ||
      (now() - last_cmd_time_).seconds() > cmd_timeout_s_) {
    vx_f = 0.0;
    vy_f = 0.0;
    omega = 0.0;
  }

  // Step 1 — Deadband (field frame, magnitude-based for xy)
  if (std::hypot(vx_f, vy_f) < deadband_xy_) {
    vx_f = 0.0;
    vy_f = 0.0;
  }
  if (std::abs(omega) < deadband_omega_) {
    omega = 0.0;
  }

  // Step 2 — Field-centric rotation
  double vx_r, vy_r;
  if (field_centric_) {
    field_to_body(vx_f, vy_f, imu_yaw_rad_, vx_r, vy_r);
  } 
  // else {
  //   // vx_r = vx_f;
  //   // vy_r = vy_f;
  // }

  // Step 3 — Mix heading-lock omega (suppress during manual rotation)
  double omega_total;
  if (std::abs(omega) < deadband_omega_) {
    omega_total = omega + heading_omega_weight_ * heading_omega_;
  } else {
    omega_total = omega;
  }

  // Step 4 — Slew-rate limiter (robot frame)
  cur_vx_ = ramp(cur_vx_, vx_r, slew_xy_ * dt_);
  cur_vy_ = ramp(cur_vy_, vy_r, slew_xy_ * dt_);
  cur_omega_ = ramp(cur_omega_, omega_total, slew_omega_ * dt_);

  // Step 5 — Second-order chassis discretization (WPILib-style)
  // Corrects sideways drift when translating+rotating simultaneously by
  // applying the pose-exponential of the twist over one timestep.
  double vx_out = cur_vx_;
  double vy_out = cur_vy_;
  double omega_out = cur_omega_;
  const double dtheta = omega_out * dt_;
  if (std::abs(dtheta) > 1e-6) {
    const double s = std::sin(dtheta) / dtheta;
    const double c = (1.0 - std::cos(dtheta)) / dtheta;
    const double vx_new = s * vx_out - c * vy_out;
    const double vy_new = c * vx_out + s * vy_out;
    vx_out = vx_new;
    vy_out = vy_new;
  }

  // Step 6 — Wheel speed desaturation
  desaturate(vx_out, vy_out, omega_out);

  // Update slew state to desaturated values
  cur_vx_ = vx_out;
  cur_vy_ = vy_out;
  cur_omega_ = omega_out;

  // Step 7 — Publish robot-frame /cmd_vel.
  // While a climb case is active, climb_node owns /cmd_vel (body-frame, vy
  // aligned with the climb mechanism's commanded speed). Yield to it.
  if (climb_active_.load()) return;

  geometry_msgs::msg::Twist twist;
  twist.linear.x = vx_out;   // strafe
  twist.linear.y = vy_out;   // forward
  twist.angular.z = omega_out;
  pub_cmd_->publish(twist);
}

void SwerveControllerNode::pub_diag() {
  std::ostringstream ss;
  ss << "{\"field_centric\":" << (field_centric_ ? "true" : "false")
     << ",\"imu_yaw_deg\":" << (imu_yaw_rad_ * 180.0 / M_PI)
     << ",\"vx_field\":" << vx_field_
     << ",\"vy_field\":" << vy_field_
     << ",\"omega_cmd\":" << omega_cmd_
     << ",\"heading_omega\":" << heading_omega_
     << ",\"cur_vx\":" << cur_vx_
     << ",\"cur_vy\":" << cur_vy_
     << ",\"cur_omega\":" << cur_omega_ << "}";

  std_msgs::msg::String msg;
  msg.data = ss.str();
  pub_status_->publish(msg);
}

}  // namespace world

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<world::SwerveControllerNode>());
  rclcpp::shutdown();
  return 0;
}
