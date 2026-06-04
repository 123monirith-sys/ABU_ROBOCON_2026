#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/vector3.hpp>
#include "robot_msg/msg/motor.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

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


    ttt_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/ttt_cmd",10,
            std::bind(&CanTxNode::ttt_callback, this, _1)
      );

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

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr ttt_sub_;



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


    int16_t ttt_d1_ = 0;
    int16_t ttt_d2_ = 0;
    int16_t ttt_d3_ = 0;
    int16_t ttt_d4_ = 0;

    bool ttt_updated_ = false;


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



    void ttt_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if(msg->data.size() < 4)
            return;

        std::lock_guard<std::mutex> lock(mtx_);

    ttt_d1_ = static_cast<int16_t>(msg->data[0] * 100);

    ttt_d2_ = static_cast<int16_t>(msg->data[1] * 100);

    ttt_d3_ = static_cast<int16_t>(msg->data[2] * 100);

    ttt_d4_ = static_cast<int16_t>(msg->data[3] * 100);

    ttt_updated_ = true;
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

        bool send_ttt = false;

        int16_t ttt_d1;
        int16_t ttt_d2;
        int16_t ttt_d3;
        int16_t ttt_d4;

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

            ttt_d1 = ttt_d1_;
            ttt_d2 = ttt_d2_;
            ttt_d3 = ttt_d3_;
            ttt_d4 = ttt_d4_;
            
            send_ttt = ttt_updated_;
            ttt_updated_ = false;

        }

        // ================= SMALL NOISE FILTER =================
        if (std::fabs(vx) < 0.005f) vx = 0.0f;
        if (std::fabs(vy) < 0.005f) vy = 0.0f;
        if (std::fabs(omega) < 0.005f) omega = 0.0f;

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

    if(send_ttt)
    {
        uint8_t ttt_data[8] = {0};

        std::memcpy(&ttt_data[0], &ttt_d1, 2);
        std::memcpy(&ttt_data[2], &ttt_d2, 2);
        std::memcpy(&ttt_data[4], &ttt_d3, 2);
        std::memcpy(&ttt_data[6], &ttt_d4, 2);
        send_can(0x300, ttt_data, 8);
    }



        // // ================= DEBUG =================
        // RCLCPP_INFO_THROTTLE(
        //     this->get_logger(),
        //     *this->get_clock(),
        //     500,
        //     "vx=%d vy=%d omega=%d",
        //     vx_i,
        //     vy_i,
        //     omega_i
        // );
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