#include "kinematic/ik_node.hpp"

#include <nlohmann/json.hpp>
#include <cmath>

using json = nlohmann::json;

namespace kinematic {

// Hysteresis band on the 180° drive-inversion decision (degrees).
// Once inverted, stay inverted until |diff| falls below FLIP_OFF.
static constexpr double FLIP_ON = 100.0;
static constexpr double FLIP_OFF = 80.0;

// Below this commanded wheel speed (m/s) the direction from atan2 is noise,
// so we hold the previous wheel angle and emit zero rpm.
static constexpr double MIN_SPD = 0.005;

// Collapse the unwrapped tracker to wrap180 after this long at neutral cmd.
static constexpr double AUTO_RESET_TIME = 2.5;

double wrap180(double a) {
  a = std::fmod(a + 180.0, 360.0);
  if (a < 0.0) a += 360.0;
  return a - 180.0;
}

double continuous_angle(double last_angle, double target_angle) {
  return last_angle + wrap180(target_angle - last_angle);
}

IkNode::IkNode() : Node("ik_node") {
  declare_parameter("rate_hz", 100.0);
  rate_hz_ = get_parameter("rate_hz").as_double();

  const double l2 = ROBOT_L * 0.5;
  const double w2 = ROBOT_W * 0.5;
  lx_ = {+l2, +l2, -l2, -l2};
  ly_ = {+w2, -w2, +w2, -w2};

  last_cont_angle_.fill(0.0);
  flipped_.fill(false);
  idle_timer_.fill(0.0);

  sub_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&IkNode::cmd_cb, this, std::placeholders::_1));

  pub_wheel_ = create_publisher<std_msgs::msg::String>("/wheel_cmd", 10);

  timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz_),
      std::bind(&IkNode::compute, this));

  RCLCPP_INFO(get_logger(),
              "IkNode started at %.0f Hz (L=%.3f, W=%.3f, R=%.3f)",
              rate_hz_, ROBOT_L, ROBOT_W, WHEEL_RADIUS);
}

void IkNode::cmd_cb(const geometry_msgs::msg::Twist::SharedPtr msg) {
  vx_ = msg->linear.x;
  vy_ = msg->linear.y;
  omega_ = msg->angular.z;
  have_cmd_ = true;
  last_cmd_time_ = std::chrono::steady_clock::now();
}

void IkNode::compute() {
  if (!have_cmd_) return;

  const double dt = 1.0 / rate_hz_;

  // Watchdog: if /cmd_vel goes silent, treat as neutral cmd (zero rpm, hold
  // last angles). Don't tear down state — when cmds resume, we pick up cleanly.
  const bool stale =
      (std::chrono::steady_clock::now() - last_cmd_time_) > CMD_TIMEOUT;
  if (stale) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "/cmd_vel stale (>%ldms) — holding angles, zero rpm",
                         static_cast<long>(CMD_TIMEOUT.count()));
  }

  // IK frame: vb_x = forward = linear.y; vb_y = left = linear.x;
  // omega negated to convert ROS CCW+ to STM32 CW+.
  const double vb_x = stale ? 0.0 : vy_;
  const double vb_y = stale ? 0.0 : vx_;
  const double omega = stale ? 0.0 : -omega_;

  const bool neutral = std::abs(vb_x) < 1e-6 &&
                       std::abs(vb_y) < 1e-6 &&
                       std::abs(omega) < 1e-6;

  std::array<int, 4> rpms{};
  std::array<int, 4> angles{};

  if (neutral) {
    // Hold position. After AUTO_RESET_TIME of idle, collapse the unwrapped
    // tracker to its wrap180 representation. The physical orientation is
    // preserved (the motor controller wraps modulo 360), so no wheel motion.
    for (int i = 0; i < 4; ++i) {
      rpms[i] = 0;
      idle_timer_[i] += dt;
      if (idle_timer_[i] >= AUTO_RESET_TIME) {
        last_cont_angle_[i] = wrap180(last_cont_angle_[i]);
        idle_timer_[i] = 0.0;
      }
      angles[i] = static_cast<int>(std::lround(last_cont_angle_[i]));
    }
  } else {
    for (int i = 0; i < 4; ++i) idle_timer_[i] = 0.0;

    for (int i = 0; i < 4; ++i) {
      const double wx = X_SIGN * vb_x + OMEGA_SIGN * omega * ly_[i];
      const double wy = Y_SIGN * vb_y - OMEGA_SIGN * omega * lx_[i];
      const double spd = std::hypot(wx, wy);

      if (spd < MIN_SPD) {
        // Direction is noise — hold last cmd, zero drive.
        angles[i] = static_cast<int>(std::lround(last_cont_angle_[i]));
        rpms[i] = 0;
        continue;
      }

      const double raw_ang = std::atan2(wy, wx) * 180.0 / M_PI;
      double rpm = (spd / WHEEL_RADIUS) * (60.0 / (2.0 * M_PI));

      // Flip decision against our own command tracker (NOT feedback).
      // This is what makes the "omega flips sign" case behave well:
      // raw_ang jumps 180°, we see |diff|≈180, latch inverted, fold the
      // 180° straight into the tracker — so the published angle stays
      // put and only the drive sign reverses. Hysteresis keeps the
      // decision from chattering at the boundary.
      const double last_wrapped = wrap180(last_cont_angle_[i]);
      const double diff_mag = std::abs(wrap180(raw_ang - last_wrapped));

      if (flipped_[i]) {
        if (diff_mag < FLIP_OFF) flipped_[i] = false;
      } else {
        if (diff_mag > FLIP_ON) flipped_[i] = true;
      }

      const double effective_raw =
          flipped_[i] ? wrap180(raw_ang + 180.0) : raw_ang;
      const double final_rpm = flipped_[i] ? -rpm : rpm;

      last_cont_angle_[i] =
          continuous_angle(last_cont_angle_[i], effective_raw);

      angles[i] = static_cast<int>(std::lround(last_cont_angle_[i]));
      rpms[i] = static_cast<int>(std::lround(final_rpm));
    }
  }

  json j;
  j["rpms"] = {rpms[0], rpms[1], rpms[2], rpms[3]};
  j["angles"] = {angles[0], angles[1], angles[2], angles[3]};

  std_msgs::msg::String msg;
  msg.data = j.dump();
  pub_wheel_->publish(msg);
}

}  // namespace kinematic

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<kinematic::IkNode>());
  rclcpp::shutdown();
  return 0;
}
