#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace kinematic {

class SwerveCanNode : public rclcpp::Node {
public:
  SwerveCanNode();
  ~SwerveCanNode() override;

private:
  // CAN constants
  static constexpr const char *CAN_INTERFACE = "can0";
  static constexpr uint32_t CAN_ID_KEEPALIVE   = 0x012;
  static constexpr uint32_t CAN_ID_MOTOR_SPEED = 0x001;
  static constexpr uint32_t CAN_ID_MOTOR_ANGLE = 0x002;
  static constexpr uint32_t CAN_ID_ENC_XY      = 0x100;
  static constexpr uint32_t CAN_ID_IMU_YAW     = 0x101;
  static constexpr int CMD_SEND_HZ  = 100;
  static constexpr int KEEPALIVE_HZ = 10;
  static constexpr int STATUS_HZ    = 2;
  static constexpr uint8_t MODE_PASSTHROUGH = 3;

  // Motor feedback CAN IDs → wheel index
  static int motor_fb_index(uint32_t can_id);

  static uint8_t xor_checksum(const uint8_t *data, size_t len);

  // CAN I/O
  void can_send(uint32_t can_id, const uint8_t *data, size_t len);
  void rx_loop();
  bool open_can();
  void close_can();
  void schedule_reopen();
  void maybe_reopen();

  // Callbacks
  void wheel_cmd_cb(const std_msgs::msg::String::SharedPtr msg);

  // Timers
  void send_keepalive();
  void send_motor_cmd();
  void publish_rx_data();
  void pub_diag();

  // Shutdown helper
  void send_stop();

  // SocketCAN
  std::atomic<int> can_fd_{-1};
  std::atomic<bool> need_reopen_{false};
  std::chrono::steady_clock::time_point next_reopen_attempt_{};
  std::atomic<int64_t> last_rx_ns_{0};
  bool ever_had_rx_{false};
  std::mutex lock_;
  std::atomic<bool> running_{true};
  std::thread rx_thread_;

  // Wheel commands
  std::array<int16_t, 4> rpms_{};
  std::array<int16_t, 4> angles_{};
  bool have_cmd_ = false;
  // Watchdog: zero motor commands if /wheel_cmd goes silent.
  std::chrono::steady_clock::time_point last_cmd_time_{};
  static constexpr std::chrono::milliseconds CMD_TIMEOUT{200};

  // Motor feedback
  std::array<int16_t, 4> fb_angles_{};
  std::array<int16_t, 4> fb_speeds_{};

  // Odom / IMU from CAN
  double robotx_ = 0.0, roboty_ = 0.0;
  double yaw_deg_ = 0.0;

  // Pending publish data (set by RX thread)
  struct PendingOdom { double x, y; };
  std::optional<PendingOdom> pending_odom_;
  std::optional<double> pending_yaw_;
  bool pending_fb_ = false;

  // Diagnostics
  uint64_t tx_count_ = 0, rx_count_ = 0;
  std::string last_err_;
  int tx_err_count_ = 0;

  // ROS I/O
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_wheel_cmd_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_climb_active_;
  std::atomic<bool> climb_active_{false};
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pub_odom_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_yaw_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_fb_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;

  rclcpp::TimerBase::SharedPtr timer_send_cmd_;
  rclcpp::TimerBase::SharedPtr timer_keepalive_;
  rclcpp::TimerBase::SharedPtr timer_publish_rx_;
  rclcpp::TimerBase::SharedPtr timer_diag_;
};

}  // namespace kinematic
