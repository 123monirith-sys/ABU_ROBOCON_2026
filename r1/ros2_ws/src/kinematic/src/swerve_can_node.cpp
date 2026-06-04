#include "kinematic/swerve_can_node.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cstring>

using json = nlohmann::json;

namespace kinematic {

// Motor feedback CAN IDs: 0x200=FL, 0x202=FR, 0x204=BL, 0x206=BR
int SwerveCanNode::motor_fb_index(uint32_t can_id) {
  switch (can_id) {
    case 0x200: return 0;
    case 0x202: return 1;
    case 0x204: return 2;
    case 0x206: return 3;
    default:    return -1;
  }
}

uint8_t SwerveCanNode::xor_checksum(const uint8_t *data, size_t len) {
  uint8_t v = 0;
  for (size_t i = 0; i < len; ++i) v ^= data[i];
  return v;
}

SwerveCanNode::SwerveCanNode() : Node("swerve_can_node") {
  RCLCPP_INFO(get_logger(), "Opening CAN: %s", CAN_INTERFACE);

  if (!open_can()) {
    RCLCPP_WARN(get_logger(),
                "CAN open failed at startup; will retry. Is %s up?",
                CAN_INTERFACE);
    schedule_reopen();
  }

  // ROS I/O
  sub_wheel_cmd_ = create_subscription<std_msgs::msg::String>(
      "/wheel_cmd", 10,
      std::bind(&SwerveCanNode::wheel_cmd_cb, this, std::placeholders::_1));

  rclcpp::QoS climb_qos(1);
  climb_qos.transient_local();
  sub_climb_active_ = create_subscription<std_msgs::msg::Bool>(
      "/climb/active", climb_qos,
      [this](const std_msgs::msg::Bool::SharedPtr m) {
        climb_active_.store(m->data);
      });

  pub_odom_ = create_publisher<geometry_msgs::msg::Pose2D>("/odom_enc", 10);
  pub_yaw_ = create_publisher<std_msgs::msg::Float32>("/imu_yaw", 10);
  pub_fb_ = create_publisher<std_msgs::msg::String>("/wheel_feedback", 10);
  pub_status_ = create_publisher<std_msgs::msg::String>("/can/status", 10);

  // Timers
  timer_send_cmd_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / CMD_SEND_HZ),
      std::bind(&SwerveCanNode::send_motor_cmd, this));
  timer_keepalive_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / KEEPALIVE_HZ),
      std::bind(&SwerveCanNode::send_keepalive, this));
  timer_publish_rx_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / CMD_SEND_HZ),
      std::bind(&SwerveCanNode::publish_rx_data, this));
  timer_diag_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / STATUS_HZ),
      std::bind(&SwerveCanNode::pub_diag, this));

  // CAN RX thread
  rx_thread_ = std::thread(&SwerveCanNode::rx_loop, this);

  RCLCPP_INFO(get_logger(), "SwerveCanNode ready (PASSTHROUGH mode).");
}

SwerveCanNode::~SwerveCanNode() {
  running_ = false;
  if (rx_thread_.joinable()) rx_thread_.join();

  send_stop();

  close_can();
}

// ─── CAN I/O ─────────────────────────────────────────────────────────

bool SwerveCanNode::open_can() {
  int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd < 0) return false;

  struct ifreq ifr{};
  std::strncpy(ifr.ifr_name, CAN_INTERFACE, IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';
  if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) { close(fd); return false; }

  struct sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    close(fd);
    return false;
  }

  struct timeval tv{0, 100000};
  setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  close_can();
  can_fd_ = fd;
  need_reopen_ = false;
  RCLCPP_INFO(get_logger(), "CAN %s opened (fd=%d)", CAN_INTERFACE, fd);
  return true;
}

void SwerveCanNode::close_can() {
  int fd = can_fd_.exchange(-1);
  if (fd >= 0) close(fd);
}

void SwerveCanNode::schedule_reopen() {
  need_reopen_ = true;
  next_reopen_attempt_ =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
  close_can();
}

void SwerveCanNode::maybe_reopen() {
  const auto now = std::chrono::steady_clock::now();
  const int64_t now_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          now.time_since_epoch()).count();

  const int64_t last_rx = last_rx_ns_.load(std::memory_order_relaxed);
  if (last_rx != 0) ever_had_rx_ = true;
  if (ever_had_rx_ && can_fd_ >= 0 && !need_reopen_ &&
      (now_ns - last_rx) > 2'000'000'000LL) {
    RCLCPP_WARN(get_logger(),
                "No CAN RX for >2s — forcing socket reopen");
    schedule_reopen();
    last_rx_ns_.store(now_ns, std::memory_order_relaxed);
  }

  if (need_reopen_ && now >= next_reopen_attempt_) {
    if (!open_can()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "CAN reopen failed, will retry");
      next_reopen_attempt_ = now + std::chrono::milliseconds(500);
    } else {
      last_rx_ns_.store(now_ns, std::memory_order_relaxed);
    }
  }
}

void SwerveCanNode::can_send(uint32_t can_id, const uint8_t *data,
                              size_t len) {
  if (can_fd_ < 0) return;
  struct can_frame frame{};
  frame.can_id = can_id;
  frame.can_dlc = static_cast<uint8_t>(len);
  std::memcpy(frame.data, data, len);

  if (write(can_fd_, &frame, sizeof(frame)) < 0) {
    int e = errno;
    ++tx_err_count_;
    last_err_ = strerror(e);
    if (tx_err_count_ <= 1) {
      RCLCPP_WARN(get_logger(), "CAN TX error: %s", last_err_.c_str());
    }
    if (e == ENXIO || e == ENODEV || e == ENETDOWN ||
        e == EBADF || e == ENOBUFS) {
      schedule_reopen();
    }
  } else {
    ++tx_count_;
    if (tx_err_count_ > 0) {
      RCLCPP_INFO(get_logger(), "CAN TX recovered.");
      tx_err_count_ = 0;
    }
  }
}

void SwerveCanNode::rx_loop() {
  struct can_frame frame;
  while (running_) {
    int fd = can_fd_;
    if (fd < 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }
    ssize_t nbytes = read(fd, &frame, sizeof(frame));
    if (nbytes < 0) {
      int e = errno;
      if (e == EAGAIN || e == EWOULDBLOCK) continue;  // timeout
      if (running_) {
        std::lock_guard<std::mutex> lk(lock_);
        last_err_ = strerror(e);
      }
      if (e == ENXIO || e == ENODEV || e == ENETDOWN || e == EBADF) {
        schedule_reopen();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      continue;
    }
    if (nbytes < static_cast<ssize_t>(sizeof(frame))) continue;

    last_rx_ns_.store(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count(),
        std::memory_order_relaxed);

    const uint32_t aid = frame.can_id & CAN_EFF_MASK;
    const uint8_t *d = frame.data;
    const size_t dlen = frame.can_dlc;

    std::lock_guard<std::mutex> lk(lock_);
    ++rx_count_;

    if (aid == CAN_ID_ENC_XY && dlen >= 8) {
      float rx, ry;
      std::memcpy(&rx, d + 0, 4);
      std::memcpy(&ry, d + 4, 4);
      robotx_ = rx;
      roboty_ = ry;
      pending_odom_ = {rx, ry};

    } else if (aid == CAN_ID_IMU_YAW && dlen >= 4) {
      float yaw;
      std::memcpy(&yaw, d, 4);
      yaw_deg_ = yaw;
      pending_yaw_ = static_cast<double>(yaw);

    } else {
      int idx = motor_fb_index(aid);
      if (idx >= 0 && dlen >= 4) {
        int16_t angle, speed;
        std::memcpy(&angle, d + 0, 2);
        std::memcpy(&speed, d + 2, 2);
        fb_angles_[idx] = angle;
        fb_speeds_[idx] = speed;
        pending_fb_ = true;
      }
    }
  }
}

// ─── Callbacks ───────────────────────────────────────────────────────

void SwerveCanNode::wheel_cmd_cb(
    const std_msgs::msg::String::SharedPtr msg) {
  try {
    auto j = json::parse(msg->data);
    auto rpms = j.at("rpms").get<std::vector<int>>();
    auto angles = j.at("angles").get<std::vector<int>>();
    if (rpms.size() == 4 && angles.size() == 4) {
      std::lock_guard<std::mutex> lk(lock_);
      for (int i = 0; i < 4; ++i) {
        rpms_[i] = static_cast<int16_t>(rpms[i]);
        angles_[i] = static_cast<int16_t>(angles[i]);
      }
      have_cmd_ = true;
      last_cmd_time_ = std::chrono::steady_clock::now();
    }
  } catch (const json::exception &) {
    // ignore malformed
  }
}

// ─── TX Timers ───────────────────────────────────────────────────────

void SwerveCanNode::send_keepalive() {
  uint8_t payload[8] = {0, 0, 0, 0, 0, 0, MODE_PASSTHROUGH, 0};
  payload[7] = xor_checksum(payload, 7);
  can_send(CAN_ID_KEEPALIVE, payload, 8);
}

void SwerveCanNode::send_motor_cmd() {
  maybe_reopen();
  int16_t rpms[4], angs[4];
  bool stale;
  {
    std::lock_guard<std::mutex> lk(lock_);
    if (!have_cmd_) return;
    stale = (std::chrono::steady_clock::now() - last_cmd_time_) > CMD_TIMEOUT;
    if (stale) {
      // Zero drive but keep last commanded angles — wheels stop in place.
      std::fill(std::begin(rpms), std::end(rpms), int16_t{0});
    } else {
      std::memcpy(rpms, rpms_.data(), 8);
    }
    std::memcpy(angs, angles_.data(), 8);
  }

  if (stale) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "/wheel_cmd stale (>%ldms) — sending zero RPM",
                         static_cast<long>(CMD_TIMEOUT.count()));
  }

  // 4x int16 LE for speeds
  can_send(CAN_ID_MOTOR_SPEED,
           reinterpret_cast<const uint8_t *>(rpms), 8);
  // 4x int16 LE for angles
  can_send(CAN_ID_MOTOR_ANGLE,
           reinterpret_cast<const uint8_t *>(angs), 8);
}

// ─── Publish RX Data ─────────────────────────────────────────────────

void SwerveCanNode::publish_rx_data() {
  std::optional<PendingOdom> odom;
  std::optional<double> yaw;
  bool fb = false;
  std::array<int16_t, 4> fb_a, fb_s;

  {
    std::lock_guard<std::mutex> lk(lock_);
    odom = pending_odom_;
    yaw = pending_yaw_;
    fb = pending_fb_;
    fb_a = fb_angles_;
    fb_s = fb_speeds_;
    pending_odom_.reset();
    pending_yaw_.reset();
    pending_fb_ = false;
  }

  if (odom.has_value() && !climb_active_.load()) {
    geometry_msgs::msg::Pose2D msg;
    msg.x = odom->x;
    msg.y = odom->y;
    // theta intentionally left at 0 — yaw is delivered separately on /imu_yaw.
    pub_odom_->publish(msg);
  }

  if (yaw.has_value()) {
    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(yaw.value());
    pub_yaw_->publish(msg);
  }

  if (fb) {
    json j;
    j["angles"] = {fb_a[0], fb_a[1], fb_a[2], fb_a[3]};
    j["speeds"] = {fb_s[0], fb_s[1], fb_s[2], fb_s[3]};
    std_msgs::msg::String msg;
    msg.data = j.dump();
    pub_fb_->publish(msg);
  }
}

// ─── Diagnostics ─────────────────────────────────────────────────────

void SwerveCanNode::pub_diag() {
  std::lock_guard<std::mutex> lk(lock_);

  json j;
  j["mode"] = "PASSTHROUGH";
  j["tx_count"] = tx_count_;
  j["rx_count"] = rx_count_;
  j["rpms"] = {rpms_[0], rpms_[1], rpms_[2], rpms_[3]};
  j["angles"] = {angles_[0], angles_[1], angles_[2], angles_[3]};
  j["robotx"] = robotx_;
  j["roboty"] = roboty_;
  j["yaw_deg"] = yaw_deg_;
  j["fb_angles"] = {fb_angles_[0], fb_angles_[1], fb_angles_[2], fb_angles_[3]};
  j["fb_speeds"] = {fb_speeds_[0], fb_speeds_[1], fb_speeds_[2], fb_speeds_[3]};
  j["last_err"] = last_err_;

  std_msgs::msg::String msg;
  msg.data = j.dump();
  pub_status_->publish(msg);
}

// ─── Shutdown ────────────────────────────────────────────────────────

void SwerveCanNode::send_stop() {
  if (can_fd_ < 0) return;

  // Send STOP keepalive (mode=0)
  uint8_t payload[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  payload[7] = xor_checksum(payload, 7);
  can_send(CAN_ID_KEEPALIVE, payload, 8);

  // Zero motor commands
  int16_t zero[4] = {0, 0, 0, 0};
  can_send(CAN_ID_MOTOR_SPEED,
           reinterpret_cast<const uint8_t *>(zero), 8);

  RCLCPP_INFO(get_logger(), "Final STOP sent.");
}

}  // namespace kinematic

int main(int argc, char **argv) {
  // Note: CAN interface must be brought up before running:
  //   sudo ip link set can0 up type can bitrate 1000000
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<kinematic::SwerveCanNode>());
  } catch (const std::runtime_error &e) {
    RCLCPP_FATAL(rclcpp::get_logger("swerve_can_node"), "%s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
