#include "kinematic/ekf_node.hpp"

#include <sstream>

namespace kinematic {

double EkfNode::wrap_angle(double a) {
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a < 0.0) a += 2.0 * M_PI;
  return a - M_PI;
}

EkfNode::EkfNode() : Node("ekf_node") {
  declare_parameter("rate_hz", 50.0);
  declare_parameter("q_xy", 0.001);
  declare_parameter("q_theta", 0.0005);
  declare_parameter("r_imu", 0.01);
  declare_parameter("r_enc_xy", 0.005);
  declare_parameter("use_encoder_update", false);

  rate_hz_ = get_parameter("rate_hz").as_double();
  q_xy_ = get_parameter("q_xy").as_double();
  q_theta_ = get_parameter("q_theta").as_double();
  r_imu_ = get_parameter("r_imu").as_double();
  r_enc_xy_ = get_parameter("r_enc_xy").as_double();
  use_enc_update_ = get_parameter("use_encoder_update").as_bool();
  dt_ = 1.0 / rate_hz_;

  // Initialize state
  x_ = {0.0, 0.0, 0.0};

  P_ = {{{0.1, 0.0, 0.0},
          {0.0, 0.1, 0.0},
          {0.0, 0.0, 0.1}}};

  Q_ = {{{q_xy_, 0.0, 0.0},
          {0.0, q_xy_, 0.0},
          {0.0, 0.0, q_theta_}}};

  sub_odom_ = create_subscription<geometry_msgs::msg::Pose2D>(
      "/odom_enc", 10,
      std::bind(&EkfNode::odom_cb, this, std::placeholders::_1));
  sub_imu_ = create_subscription<std_msgs::msg::Float32>(
      "/imu_yaw", 10,
      std::bind(&EkfNode::imu_cb, this, std::placeholders::_1));
  sub_vel_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&EkfNode::vel_cb, this, std::placeholders::_1));
  sub_set_pose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/ekf_node/set_pose", 10,
      std::bind(&EkfNode::set_pose_cb, this, std::placeholders::_1));

  pub_pose_ = create_publisher<geometry_msgs::msg::Pose2D>("/ekf/pose", 10);
  pub_status_ = create_publisher<std_msgs::msg::String>("/ekf/status", 10);

  timer_ = create_wall_timer(
      std::chrono::duration<double>(dt_),
      std::bind(&EkfNode::step, this));

  RCLCPP_INFO(get_logger(),
              "EkfNode started at %.0f Hz (Q_xy=%.4f, Q_th=%.4f, R_imu=%.3f)",
              rate_hz_, q_xy_, q_theta_, r_imu_);
}

void EkfNode::odom_cb(const geometry_msgs::msg::Pose2D::SharedPtr msg) {
  enc_x_ = msg->x;
  enc_y_ = msg->y;
}

void EkfNode::imu_cb(const std_msgs::msg::Float32::SharedPtr msg) {
  imu_yaw_rad_ = msg->data * M_PI / 180.0;
}

void EkfNode::vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg) {
  vx_ = msg->linear.x;       // strafe
  vy_ = msg->linear.y;       // forward
  omega_z_ = msg->angular.z; // yaw rate (rad/s, ROS CCW+)
}

void EkfNode::set_pose_cb(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  const double qz = msg->pose.pose.orientation.z;
  const double qw = msg->pose.pose.orientation.w;
  const double yaw = std::atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz);

  x_[0] = msg->pose.pose.position.x;
  x_[1] = msg->pose.pose.position.y;
  x_[2] = wrap_angle(yaw);

  // Drop encoder delta history so the next /odom_enc tick doesn't apply a
  // stale (pre-reset) delta on top of the new state.
  prev_enc_x_.reset();
  prev_enc_y_.reset();
  enc_x_.reset();
  enc_y_.reset();

  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) P_[i][j] = 0.0;
  P_[0][0] = std::max(msg->pose.covariance[0],  1e-6);
  P_[1][1] = std::max(msg->pose.covariance[7],  1e-6);
  P_[2][2] = std::max(msg->pose.covariance[35], 1e-6);

  RCLCPP_INFO(get_logger(), "set_pose: x=%.3f y=%.3f yaw=%.2f deg",
              x_[0], x_[1], x_[2] * 180.0 / M_PI);
}

void EkfNode::step() {
  const double theta = x_[2];
  bool use_jacobian = false;
  double F02 = 0.0, F12 = 0.0;  // only non-trivial Jacobian entries

  // --- Predict using encoder deltas (preferred) ---
  // Encoder deltas are BODY-frame (rotary wheels rigidly mounted on robot).
  // Rotate by *midpoint* heading (theta + 0.5*omega*dt) to get world-frame
  // displacement. Midpoint integration matters when the robot is rotating:
  // using the start-of-step theta consistently leans the path in the rotation
  // direction. omega_z_ is the commanded yaw rate (no IMU yaw rate available
  // here), good enough for the small correction.
  const double theta_mid = theta + 0.5 * omega_z_ * dt_;
  const double cos_th = std::cos(theta_mid);
  const double sin_th = std::sin(theta_mid);

  if (enc_x_.has_value() && prev_enc_x_.has_value()) {
    const double bdx = enc_x_.value() - prev_enc_x_.value();  // body-frame Δstrafe
    const double bdy = enc_y_.value() - prev_enc_y_.value();  // body-frame Δforward
    x_[0] += cos_th * bdx - sin_th * bdy;
    x_[1] += sin_th * bdx + cos_th * bdy;
    // Keep F = I on this path. IMU is authoritative for theta; if we also
    // feed theta uncertainty back through F02/F12, every IMU yaw update
    // kicks (x, y) via the cross-covariance P[0..1][2] and the pose shakes
    // during rotation.
  } else if (std::abs(vy_) > 1e-9 || std::abs(vx_) > 1e-9) {
    // Fallback: dead-reckon from cmd_vel (body-frame velocities)
    x_[0] += (cos_th * vx_ - sin_th * vy_) * dt_;
    x_[1] += (sin_th * vx_ + cos_th * vy_) * dt_;

    F02 = (-sin_th * vx_ - cos_th * vy_) * dt_;
    F12 = ( cos_th * vx_ - sin_th * vy_) * dt_;
    use_jacobian = true;
  }

  if (enc_x_.has_value()) {
    prev_enc_x_ = enc_x_;
    prev_enc_y_ = enc_y_;
  }

  // Propagate covariance: P = F * P * F^T + Q
  if (use_jacobian) {
    // F is identity except F[0][2]=F02, F[1][2]=F12
    // Compute F*P*F^T in-place using the sparse structure
    // Step 1: P' = F * P  (only rows 0,1 change)
    // P'[0][j] = P[0][j] + F02 * P[2][j]
    // P'[1][j] = P[1][j] + F12 * P[2][j]
    std::array<double, 3> P0 = P_[0], P1 = P_[1];
    for (int j = 0; j < 3; ++j) {
      P_[0][j] = P0[j] + F02 * P_[2][j];
      P_[1][j] = P1[j] + F12 * P_[2][j];
    }
    // Step 2: P'' = P' * F^T  (only columns 0,1 change, but F^T[2][0]=F02, F^T[2][1]=F12)
    // P''[i][0] = P'[i][0]  (unchanged, F^T col 0 = [1,0,0])
    // P''[i][1] = P'[i][1]  (unchanged, F^T col 1 = [0,1,0])
    // P''[i][2] = P'[i][2] + P'[i][0]*F02 + P'[i][1]*F12
    for (int i = 0; i < 3; ++i) {
      P_[i][2] += P_[i][0] * F02 + P_[i][1] * F12;
    }
    // Add Q
    for (int i = 0; i < 3; ++i) P_[i][i] += Q_[i][i];
  } else {
    // Encoder path: F = I, so P = P + Q
    for (int i = 0; i < 3; ++i) P_[i][i] += Q_[i][i];
  }

  // --- Update with IMU yaw ---
  if (imu_yaw_rad_.has_value()) {
    const double z = imu_yaw_rad_.value();
    const double y_innov = wrap_angle(z - x_[2]);
    const double S = P_[2][2] + r_imu_;

    if (std::abs(S) > 1e-12) {
      std::array<double, 3> K;
      for (int i = 0; i < 3; ++i) {
        K[i] = P_[i][2] / S;
      }

      for (int i = 0; i < 3; ++i) {
        x_[i] += K[i] * y_innov;
      }
      x_[2] = wrap_angle(x_[2]);

      // P = (I - K*H) * P,  H = [0, 0, 1]
      // Must use a copy of P[2][j] since we're modifying P in-place
      std::array<double, 3> P2_row = {P_[2][0], P_[2][1], P_[2][2]};
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          P_[i][j] -= K[i] * P2_row[j];
        }
      }
    }
  }

  // --- Optional: update with encoder absolute position ---
  // WARNING: encoders publish BODY-frame cumulative distance, not world
  // position. Fusing them as world-frame measurements is only valid if the
  // robot has never rotated. Leave disabled unless you know what you're doing.
  if (use_enc_update_ && enc_x_.has_value()) {
    for (int axis = 0; axis < 2; ++axis) {
      const double z = (axis == 0) ? enc_x_.value() : enc_y_.value();
      const double y_innov = z - x_[axis];
      const double S = P_[axis][axis] + r_enc_xy_;

      if (std::abs(S) > 1e-12) {
        std::array<double, 3> K;
        for (int i = 0; i < 3; ++i) {
          K[i] = P_[i][axis] / S;
        }

        for (int i = 0; i < 3; ++i) {
          x_[i] += K[i] * y_innov;
        }
        x_[2] = wrap_angle(x_[2]);

        std::array<double, 3> Pa_row = {P_[axis][0], P_[axis][1], P_[axis][2]};
        for (int i = 0; i < 3; ++i) {
          for (int j = 0; j < 3; ++j) {
            P_[i][j] -= K[i] * Pa_row[j];
          }
        }
      }
    }
  }

  // --- Publish ---
  geometry_msgs::msg::Pose2D pose;
  pose.x = x_[0];
  pose.y = x_[1];
  pose.theta = x_[2];
  pub_pose_->publish(pose);

  std::ostringstream ss;
  ss << "{\"x\":" << x_[0]
     << ",\"y\":" << x_[1]
     << ",\"theta_deg\":" << (x_[2] * 180.0 / M_PI)
     << ",\"P_diag\":[" << P_[0][0] << "," << P_[1][1] << "," << P_[2][2]
     << "]}";

  std_msgs::msg::String status;
  status.data = ss.str();
  pub_status_->publish(status);
}

}  // namespace kinematic

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<kinematic::EkfNode>());
  rclcpp::shutdown();
  return 0;
}
