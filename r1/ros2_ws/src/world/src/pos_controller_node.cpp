#include "world/pos_controller_node.hpp"

#include <algorithm>
#include <sstream>

namespace world {

double PosControllerNode::wrap_pi(double a) {
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a < 0.0) a += 2.0 * M_PI;
  return a - M_PI;
}

PosControllerNode::PosControllerNode() : Node("pos_controller_node") {
  declare_parameter("rate_hz", 100.0);
  declare_parameter("kp_xy", 0.8);
  declare_parameter("kd_xy", 0.1);
  declare_parameter("kp_xy_near", 0.5);
  declare_parameter("kd_xy_near", 0.2);
  declare_parameter("near_thresh", 0.3);
  declare_parameter("kp_theta", 1.5);
  declare_parameter("kd_theta", 0.1);
  declare_parameter("dead_xy", 0.02);
  declare_parameter("dead_theta_deg", 1.0);
  declare_parameter("arrive_hold_xy", 0.05);
  declare_parameter("arrive_hold_theta_deg", 5.0);
  declare_parameter("max_v", 1.5);
  declare_parameter("max_w", 1.5);
  declare_parameter("deriv_alpha", 0.3);

  rate_hz_ = get_parameter("rate_hz").as_double();
  kp_xy_ = get_parameter("kp_xy").as_double();
  kd_xy_ = get_parameter("kd_xy").as_double();
  kp_xy_near_ = get_parameter("kp_xy_near").as_double();
  kd_xy_near_ = get_parameter("kd_xy_near").as_double();
  near_thresh_ = get_parameter("near_thresh").as_double();
  kp_theta_ = get_parameter("kp_theta").as_double();
  kd_theta_ = get_parameter("kd_theta").as_double();
  dead_xy_ = get_parameter("dead_xy").as_double();
  dead_theta_ = get_parameter("dead_theta_deg").as_double() * M_PI / 180.0;
  hold_xy_ = get_parameter("arrive_hold_xy").as_double();
  hold_theta_ = get_parameter("arrive_hold_theta_deg").as_double() * M_PI / 180.0;
  max_v_ = get_parameter("max_v").as_double();
  max_w_ = get_parameter("max_w").as_double();
  alpha_ = get_parameter("deriv_alpha").as_double();
  dt_ = 1.0 / rate_hz_;

  param_cb_handle_ = add_on_set_parameters_callback(
      std::bind(&PosControllerNode::on_params, this, std::placeholders::_1));

  sub_target_ = create_subscription<geometry_msgs::msg::Pose2D>(
      "/target_pose", 10,
      std::bind(&PosControllerNode::target_cb, this, std::placeholders::_1));
  sub_pose_ = create_subscription<geometry_msgs::msg::Pose2D>(
      "/ekf/pose", 10,
      std::bind(&PosControllerNode::pose_cb, this, std::placeholders::_1));

  pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>("/field_cmd_vel", 10);
  pub_arrived_ = create_publisher<std_msgs::msg::Bool>("/target_arrived", 10);
  pub_status_ = create_publisher<std_msgs::msg::String>("/pos_ctrl/status", 5);

  timer_loop_ = create_wall_timer(
      std::chrono::duration<double>(dt_),
      std::bind(&PosControllerNode::loop, this));
  timer_diag_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&PosControllerNode::pub_diag, this));

  RCLCPP_INFO(get_logger(), "PosControllerNode started at %.0f Hz -> /field_cmd_vel",
              rate_hz_);
}

rcl_interfaces::msg::SetParametersResult
PosControllerNode::on_params(const std::vector<rclcpp::Parameter> &params) {
  for (const auto &p : params) {
    const auto &n = p.get_name();
    if      (n == "kp_xy")                 kp_xy_ = p.as_double();
    else if (n == "kd_xy")                 kd_xy_ = p.as_double();
    else if (n == "kp_xy_near")            kp_xy_near_ = p.as_double();
    else if (n == "kd_xy_near")            kd_xy_near_ = p.as_double();
    else if (n == "near_thresh")           near_thresh_ = p.as_double();
    else if (n == "kp_theta")              kp_theta_ = p.as_double();
    else if (n == "kd_theta")              kd_theta_ = p.as_double();
    else if (n == "dead_xy")               dead_xy_ = p.as_double();
    else if (n == "dead_theta_deg")        dead_theta_ = p.as_double() * M_PI / 180.0;
    else if (n == "arrive_hold_xy")        hold_xy_ = p.as_double();
    else if (n == "arrive_hold_theta_deg") hold_theta_ = p.as_double() * M_PI / 180.0;
    else if (n == "max_v")                 max_v_ = p.as_double();
    else if (n == "max_w")                 max_w_ = p.as_double();
    else if (n == "deriv_alpha")           alpha_ = p.as_double();
  }
  rcl_interfaces::msg::SetParametersResult res;
  res.successful = true;
  return res;
}

void PosControllerNode::target_cb(
    const geometry_msgs::msg::Pose2D::SharedPtr msg) {
  const double new_x = msg->x;
  const double new_y = msg->y;
  const double new_theta = msg->theta * M_PI / 180.0;  // theta arrives in degrees

  if (!prev_target_.has_value() ||
      std::abs(new_x - std::get<0>(prev_target_.value())) > 1e-4 ||
      std::abs(new_y - std::get<1>(prev_target_.value())) > 1e-4 ||
      std::abs(wrap_pi(new_theta - std::get<2>(prev_target_.value()))) >
          0.5 * M_PI / 180.0) {
    pd_reset_ = true;
    arrived_ = false;
  }

  x_ref_ = new_x;
  y_ref_ = new_y;
  theta_ref_ = new_theta;
  prev_target_ = std::make_tuple(new_x, new_y, new_theta);
  have_target_ = true;
}

void PosControllerNode::pose_cb(
    const geometry_msgs::msg::Pose2D::SharedPtr msg) {
  X_ = msg->x;
  Y_ = msg->y;
  theta_ = msg->theta;  // radians from EKF
  have_pose_ = true;
}

void PosControllerNode::loop() {
  if (!(have_target_ && have_pose_)) {
    return;
  }

  const double ex = x_ref_ - X_;
  const double ey = y_ref_ - Y_;
  const double et = wrap_pi(theta_ref_ - theta_);
  const double dist = std::hypot(ex, ey);

  last_ex_ = ex;
  last_ey_ = ey;
  last_et_ = et;
  last_dist_ = dist;

  // Arrival hysteresis
  if (arrived_) {
    if (dist > hold_xy_ || std::abs(et) > hold_theta_) {
      arrived_ = false;
      pd_reset_ = true;
    } else {
      pub_zero_cmd(true);
      return;
    }
  }

  // D-kick prevention
  if (pd_reset_) {
    prev_ex_ = ex;
    prev_ey_ = ey;
    prev_et_ = et;
    filt_dex_ = 0.0;
    filt_dey_ = 0.0;
    filt_det_ = 0.0;
    pd_reset_ = false;
  }

  // Filtered derivatives (world frame)
  const double raw_dex = (ex - prev_ex_) / dt_;
  const double raw_dey = (ey - prev_ey_) / dt_;
  const double raw_det = (et - prev_et_) / dt_;
  filt_dex_ = alpha_ * raw_dex + (1.0 - alpha_) * filt_dex_;
  filt_dey_ = alpha_ * raw_dey + (1.0 - alpha_) * filt_dey_;
  filt_det_ = alpha_ * raw_det + (1.0 - alpha_) * filt_det_;
  prev_ex_ = ex;
  prev_ey_ = ey;
  prev_et_ = et;

  // Adaptive gains by distance
  double kp, kd;
  if (dist < near_thresh_) {
    kp = kp_xy_near_;
    kd = kd_xy_near_;
  } else {
    kp = kp_xy_;
    kd = kd_xy_;
  }

  // PD in world frame
  double vx_w = kp * ex + kd * filt_dex_;
  double vy_w = kp * ey + kd * filt_dey_;
  double omega = kp_theta_ * et + kd_theta_ * filt_det_;

  // Clamp velocity
  const double speed = std::hypot(vx_w, vy_w);
  if (speed > max_v_) {
    const double s = max_v_ / speed;
    vx_w *= s;
    vy_w *= s;
  }
  omega = std::clamp(omega, -max_w_, max_w_);

  // No rotation gate — swerve drive translates and rotates simultaneously.
  // Field-centric transform (variant C) is correct, so position corrections
  // go in the right direction even during rotation.

  // Dead zones
  if (dist < dead_xy_) {
    vx_w = 0.0;
    vy_w = 0.0;
  }
  if (std::abs(et) < dead_theta_) {
    omega = 0.0;
  }

  // Arrival check
  if (dist < dead_xy_ && std::abs(et) < dead_theta_) {
    arrived_ = true;
  }

  last_vx_ = vx_w;
  last_vy_ = vy_w;
  last_omega_ = omega;

  geometry_msgs::msg::Twist twist;
  twist.linear.x = vx_w;
  twist.linear.y = vy_w;
  twist.angular.z = omega;
  pub_cmd_->publish(twist);

  std_msgs::msg::Bool arr;
  arr.data = arrived_;
  pub_arrived_->publish(arr);
}

void PosControllerNode::pub_zero_cmd(bool arrived) {
  geometry_msgs::msg::Twist twist;
  pub_cmd_->publish(twist);

  std_msgs::msg::Bool arr;
  arr.data = arrived;
  pub_arrived_->publish(arr);

  last_vx_ = 0.0;
  last_vy_ = 0.0;
  last_omega_ = 0.0;
}

void PosControllerNode::pub_diag() {
  std::ostringstream ss;
  ss << "{\"have_target\":" << (have_target_ ? "true" : "false")
     << ",\"have_pose\":" << (have_pose_ ? "true" : "false")
     << ",\"arrived\":" << (arrived_ ? "true" : "false")
     << ",\"target\":{\"x\":" << x_ref_
     << ",\"y\":" << y_ref_
     << ",\"theta_deg\":" << (theta_ref_ * 180.0 / M_PI) << "}"
     << ",\"pose\":{\"x\":" << X_
     << ",\"y\":" << Y_
     << ",\"theta_deg\":" << (theta_ * 180.0 / M_PI) << "}"
     << ",\"error\":{\"ex\":" << last_ex_
     << ",\"ey\":" << last_ey_
     << ",\"etheta_deg\":" << (last_et_ * 180.0 / M_PI)
     << ",\"dist\":" << last_dist_ << "}"
     << ",\"output\":{\"vx\":" << last_vx_
     << ",\"vy\":" << last_vy_
     << ",\"omega\":" << last_omega_ << "}}";

  std_msgs::msg::String msg;
  msg.data = ss.str();
  pub_status_->publish(msg);
}

}  // namespace world

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<world::PosControllerNode>());
  rclcpp::shutdown();
  return 0;
}
