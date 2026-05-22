// #include "rclcpp/rclcpp.hpp"
// #include <geometry_msgs/msg/vector3.hpp>
// #include "robot_msg/msg/motor.hpp"

// #include <linux/can.h>
// #include <linux/can/raw.h>
// #include <net/if.h>
// #include <sys/socket.h>
// #include <sys/ioctl.h>
// #include <unistd.h>

// #include <cstring>
// #include <mutex>
// #include <cmath>


// using std::placeholders::_1;
// using namespace std::chrono_literals;

// class CanTxNode : public rclcpp::Node
// {
// public:
//     CanTxNode()
//     : Node("cantx_node"),
//       can_socket_(-1),
//       vx_(0.0f),
//       vy_(0.0f),
//       omega_(0.0f),
//       last_msg_time_(this->now())
//     {
//         init_can_bus();

//         sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
//             "/cmd_vel_custom", 10,
//             std::bind(&CanTxNode::cmd_callback, this, std::placeholders::_1));
//         cmd_motor_sub = this->create_subscription<robot_msg::msg::Motor>("/motor_cmd",10, std::bind(&CanTxNode::motor_callback, this,_1));


//         timer_ = this->create_wall_timer(
//             std::chrono::milliseconds(1),   // 200 Hz
//             std::bind(&CanTxNode::send_can_loop, this));

//         RCLCPP_INFO(this->get_logger(), "CAN TX Node Started (stable version)");
//     }

//     ~CanTxNode()
//     {
//         if (can_socket_ >= 0)
//             close(can_socket_);
//     }

// private:
//     // ===== CAN =====
//     int can_socket_;

//     // ===== SHARED STATE =====
//     float vx_;
//     float vy_;
//     float omega_;
//     int16_t desire_deg =0, desire_pose =0;
//     u_int8_t sensor1=0.0, sensor2 =0.0, sensor3 =0.0;

//     std::mutex mtx_;
//     rclcpp::Time last_msg_time_;

//     // ===== ROS =====
//     rclcpp::TimerBase::SharedPtr timer_;
//     rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_;
//     rclcpp::Subscription<robot_msg::msg::Motor>::SharedPtr cmd_motor_sub;

//     // ================== CALLBACK ==================
//     void cmd_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
//     {
//         std::lock_guard<std::mutex> lock(mtx_);

//         vx_ = sanitize(msg->x);
//         vy_= sanitize(msg->y);
//         omega_ = sanitize(msg->z);

//         last_msg_time_ = this->now();
//     }

//     // ================== SAFE VALUE CHECK ==================
//     float sanitize(float v)
//     {
//         if (std::isnan(v) || std::isinf(v))
//             return 0.0f;
//         return v;
//     }

//     // ================== CAN INIT ==================
//     void init_can_bus()
//     {
//         can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
//         if (can_socket_ < 0)
//         {
//             RCLCPP_ERROR(this->get_logger(), "CAN socket failed");
//             return;
//         }

//         struct ifreq ifr{};
//         struct sockaddr_can addr{};

//         std::strncpy(ifr.ifr_name, "can0", IFNAMSIZ - 1);
//         ioctl(can_socket_, SIOCGIFINDEX, &ifr);

//         addr.can_family = AF_CAN;
//         addr.can_ifindex = ifr.ifr_ifindex;

//         if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
//         {
//             RCLCPP_ERROR(this->get_logger(), "CAN bind failed");
//             close(can_socket_);
//             can_socket_ = -1;
//         }
//     }

//     // ================== SEND FRAME ==================
//     void send_can(uint32_t id, const uint8_t *data, uint8_t len)
//     {
//         if (can_socket_ < 0)
//         {
//             init_can_bus();
//             return;
//         }

//         struct can_frame frame{};
//         frame.can_id = id;
//         frame.can_dlc = len;
//         std::memcpy(frame.data, data, len);

//         if (write(can_socket_, &frame, sizeof(frame)) < 0)
//         {
//             RCLCPP_ERROR(this->get_logger(), "CAN write failed, reconnecting...");
//             close(can_socket_);
//             can_socket_ = -1;
//             init_can_bus();
//         }
//     }

//     // ================== MAIN LOOP ==================
//     void send_can_loop()
//     {
//         float vx, vy, w;

//         // ===== THREAD SAFE COPY =====
//         {
//             std::lock_guard<std::mutex> lock(mtx_);
//             vx = vx_;
//             vy = vy_;
//             w  = omega_;
//         }

//         // ===== TIMEOUT SAFETY (VERY IMPORTANT) =====
//         // double dt_msg = (this->now() - last_msg_time_).seconds();
//         // if (dt_msg > 0.1)
//         // {
//         //     vx = 0.0f;
//         //     vy = 0.0f;
//         //     w  = 0.0f;
//         // }

//         // ===== REMOVE MICRO NOISE (IMPORTANT FOR JUMP FIX) =====
//         // if (std::fabs(vx) < 0.001f) vx = 0.0f;
//         // if (std::fabs(vy) < 0.001f) vy = 0.0f;
//         // if (std::fabs(w)  < 0.001f) w  = 0.0f;

//         // ===== SAFE SCALING =====
//         constexpr float SCALE = 1000.0f;

//         int16_t vx_i = static_cast<int16_t>(vx * SCALE);
//         int16_t vy_i = static_cast<int16_t>(vy * SCALE);
//         int16_t w_i  = static_cast<int16_t>(w  * SCALE);

//         uint8_t data[8] = {0};

//         std::memcpy(&data[0], &vx_i, 2);
//         std::memcpy(&data[2], &vy_i, 2);
//         std::memcpy(&data[4], &w_i,  2);
//         RCLCPP_INFO(this->get_logger(), "vx = %d , vy = %d , omega=%d",vx_i,vy_i, w_i);

//         send_can(0x500, data, 8);
//     }
//     void motor_callback(const robot_msg::msg::Motor::SharedPtr msg){
//         desire_deg = static_cast<int16_t>(msg->degree);   
//         desire_pose = static_cast<int16_t>(msg->pose*10000);
//         sensor1 = static_cast<u_int8_t>(msg->sensor1);
//         sensor2 = static_cast<u_int8_t>(msg->sensor2);
//         sensor3 = static_cast<u_int8_t>(msg->sensor3);
//         uint8_t data[8] = {0};
    
//         std::memcpy(&data[0], &desire_deg, 2);
//         std::memcpy(&data[2], &desire_pose, 2);
//         data[4] = sensor1;
//         data[5] = sensor2;
//         data[6] = sensor3; 
//         send_can(0x103, data , 8);
//         //RCLCPP_INFO(this->get_logger(), "Sending motor command: degree=%d, pose=%d, sensor1=%d, sensor2=%d ,sensor3=%d", desire_deg, desire_pose, sensor1, sensor2, sensor3    );


//     }

// };

// // ================== MAIN ==================
// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<CanTxNode>());
//     rclcpp::shutdown();
//     return 0;
// }
#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/vector3.hpp>
#include "robot_msg/msg/motor.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <mutex>
#include <cmath>

using std::placeholders::_1;
using namespace std::chrono_literals;

class CanTxNode : public rclcpp::Node
{
public:
    CanTxNode()
    : Node("cantx_node")
    {
        init_can();

        // ================= SUBSCRIBERS =================
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/cmd_vel_custom",
            10,
            std::bind(&CanTxNode::cmd_callback, this, _1));

        motor_sub_ = this->create_subscription<robot_msg::msg::Motor>(
            "/motor_cmd",
            10,
            std::bind(&CanTxNode::motor_callback, this, _1));

        // ================= TIMER =================
        // 5ms = 200Hz
        timer_ = this->create_wall_timer(
            5ms,
            std::bind(&CanTxNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "CAN TX NODE STARTED");
    }

    ~CanTxNode()
    {
        if (can_socket_ >= 0)
        {
            close(can_socket_);
        }
    }

private:

    // =========================================================
    // CAN
    // =========================================================
    int can_socket_ = -1;

    // =========================================================
    // ROS
    // =========================================================
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr cmd_sub_;
    rclcpp::Subscription<robot_msg::msg::Motor>::SharedPtr motor_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // =========================================================
    // SHARED DATA
    // =========================================================
    std::mutex mtx_;

    float vx_ = 0.0f;
    float vy_ = 0.0f;
    float omega_ = 0.0f;

    int16_t desire_deg_ = 0;
    int16_t desire_pose_ = 0;

    uint8_t sensor1_ = 0;
    uint8_t sensor2_ = 0;
    uint8_t sensor3_ = 0;

    bool motor_updated_ = false;

    // =========================================================
    // SANITIZE
    // =========================================================
    float sanitize(float v)
    {
        if (std::isnan(v) || std::isinf(v))
            return 0.0f;

        return v;
    }

    // =========================================================
    // INIT CAN
    // =========================================================
    void init_can()
    {
        can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);

        if (can_socket_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "CAN SOCKET FAILED");
            return;
        }

        struct ifreq ifr {};
        struct sockaddr_can addr {};

        std::strncpy(ifr.ifr_name, "can0", IFNAMSIZ - 1);

        ioctl(can_socket_, SIOCGIFINDEX, &ifr);

        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(can_socket_,
                 (struct sockaddr *)&addr,
                 sizeof(addr)) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "CAN BIND FAILED");

            close(can_socket_);
            can_socket_ = -1;
        }
    }

    // =========================================================
    // SEND CAN
    // =========================================================
    void send_can(uint32_t id, uint8_t *data, uint8_t len)
    {
        if (can_socket_ < 0)
        {
            init_can();
            return;
        }

        struct can_frame frame {};

        frame.can_id = id;
        frame.can_dlc = len;

        std::memcpy(frame.data, data, len);

        int nbytes = write(can_socket_, &frame, sizeof(frame));

        if (nbytes < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "CAN WRITE FAILED");

            close(can_socket_);
            can_socket_ = -1;
        }
    }

    // =========================================================
    // CMD CALLBACK
    // =========================================================
    void cmd_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mtx_);

        vx_ = sanitize(msg->x);
        vy_ = sanitize(msg->y);
        omega_ = sanitize(msg->z);
    }

    // =========================================================
    // MOTOR CALLBACK
    // =========================================================
    void motor_callback(const robot_msg::msg::Motor::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mtx_);

        desire_deg_  = static_cast<int16_t>(msg->degree);
        desire_pose_ = static_cast<int16_t>(msg->pose * 10000);

        sensor1_ = static_cast<uint8_t>(msg->sensor1);
        sensor2_ = static_cast<uint8_t>(msg->sensor2);
        sensor3_ = static_cast<uint8_t>(msg->sensor3);

        motor_updated_ = true;
    }

    // =========================================================
    // TIMER LOOP
    // =========================================================
    void timer_callback()
    {
        float vx, vy, omega;

        int16_t degree, pose;

        uint8_t s1, s2, s3;

        bool send_motor = false;

        // ================= THREAD SAFE COPY =================
        {
            std::lock_guard<std::mutex> lock(mtx_);

            vx = vx_;
            vy = vy_;
            omega = omega_;

            degree = desire_deg_;
            pose = desire_pose_;

            s1 = sensor1_;
            s2 = sensor2_;
            s3 = sensor3_;

            send_motor = motor_updated_;

            motor_updated_ = false;
        }

        // ================= SMALL NOISE FILTER =================
        if (std::fabs(vx) < 0.001f) vx = 0.0f;
        if (std::fabs(vy) < 0.001f) vy = 0.0f;
        if (std::fabs(omega) < 0.001f) omega = 0.0f;

        // ================= SCALE =================
        constexpr float SCALE = 1000.0f;

        int16_t vx_i = static_cast<int16_t>(vx * SCALE);
        int16_t vy_i = static_cast<int16_t>(vy * SCALE);
        int16_t omega_i = static_cast<int16_t>(omega * SCALE);

        // =====================================================
        // SEND VELOCITY FRAME
        // =====================================================
        uint8_t vel_data[8] = {0};

        std::memcpy(&vel_data[0], &vx_i, 2);
        std::memcpy(&vel_data[2], &vy_i, 2);
        std::memcpy(&vel_data[4], &omega_i, 2);

        send_can(0x500, vel_data, 8);

        // =====================================================
        // SEND MOTOR FRAME
        // =====================================================
        if (send_motor)
        {
            uint8_t motor_data[8] = {0};

            std::memcpy(&motor_data[0], &degree, 2);
            std::memcpy(&motor_data[2], &pose, 2);

            motor_data[4] = s1;
            motor_data[5] = s2;
            motor_data[6] = s3;

            send_can(0x103, motor_data, 8);
        }

        // // ================= DEBUG =================
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            500,
            "vx=%d vy=%d omega=%d",
            vx_i,
            vy_i,
            omega_i
        );
    }
};

// =========================================================
// MAIN
// =========================================================
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CanTxNode>();

    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(node);

    executor.spin();

    rclcpp::shutdown();

    return 0;
}