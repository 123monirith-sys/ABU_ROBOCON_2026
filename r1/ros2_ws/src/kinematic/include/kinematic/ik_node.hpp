#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <array>
#include <chrono>
#include <cmath>

namespace kinematic {

// Robot geometry (must match STM32 InverseKinematic.h)
inline constexpr double ROBOT_L = 0.426;
inline constexpr double ROBOT_W = 0.426;
// inline constexpr double ROBOT_L = 0.580;
// inline constexpr double ROBOT_W = 0.580;
inline constexpr double WHEEL_RADIUS = 0.039;

// Axis signs (match STM32)
inline constexpr double X_SIGN = 1.0;     // vb_x>0 = forward
inline constexpr double Y_SIGN = 1.0;     // vb_y>0 = left
inline constexpr double OMEGA_SIGN = 1.0; // omega>0 = clockwise (STM32 convention)

double wrap180(double a);
double continuous_angle(double last_angle, double target_angle);

class IkNode : public rclcpp::Node {
public:
  IkNode();

private:
  void cmd_cb(const geometry_msgs::msg::Twist::SharedPtr msg);
  void compute();

  // Parameters
  double rate_hz_;

  // Wheel positions [FL, FR, BL, BR]
  std::array<double, 4> lx_, ly_;

  // Pure continuous *command* tracker — never written from feedback.
  std::array<double, 4> last_cont_angle_;

  // Latched flip state (hysteresis on the 180° flip decision).
  std::array<bool, 4> flipped_;

  // Seconds at neutral cmd, per wheel — drives the snap-to-zero auto reset.
  std::array<double, 4> idle_timer_;

  // Latest cmd_vel + arrival time (watchdog: stale cmd → zero rpm).
  double vx_ = 0.0;     // strafe (linear.x)
  double vy_ = 0.0;     // forward (linear.y)
  double omega_ = 0.0;  // yaw (angular.z)
  bool have_cmd_ = false;
  std::chrono::steady_clock::time_point last_cmd_time_{};
  static constexpr std::chrono::milliseconds CMD_TIMEOUT{200};

  // ROS I/O
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_wheel_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace kinematic
