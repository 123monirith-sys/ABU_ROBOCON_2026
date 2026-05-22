#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
// #include <std_msgs/msg/bool.hpp>
#include "robot_msg/msg/motor.hpp"

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <unistd.h>
#include <cstring>
#include <fcntl.h>
#include <cmath>

using std::placeholders::_1;
using namespace std::chrono_literals;
class CanRxNode : public rclcpp::Node
{
public:
    CanRxNode() : Node("canrx_node")
    {
        // ===== CAN SOCKET =====
        socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (socket_ < 0)
            throw std::runtime_error("CAN socket failed");

        struct ifreq ifr{};
        std::strcpy(ifr.ifr_name, "can0");

        if (ioctl(socket_, SIOCGIFINDEX, &ifr) < 0)
            throw std::runtime_error("CAN ioctl failed");

        struct sockaddr_can addr{};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
            throw std::runtime_error("CAN bind failed");

        fcntl(socket_, F_SETFL, O_NONBLOCK);

        // ===== Publishers =====
        vel_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/cmd_vel_imu", 10);

        stm32_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/stm32_status", 10);
        grip_pub = this->create_publisher<robot_msg::msg::Motor>("/motor_rev", 10);
        // ===== Timer =====
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&CanRxNode::readCAN, this));

        last_time_ = this->get_clock()->now();
        first_ = true;

        RCLCPP_INFO(this->get_logger(), "CAN RX NODE STARTED");
    }

    ~CanRxNode()
    {
        if (socket_ >= 0) close(socket_);
    }

private:
    int socket_;
    struct can_frame frame;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr stm32_pub;
    rclcpp::Publisher<robot_msg::msg::Motor>::SharedPtr grip_pub;


    float last_x = 0.0f;
    float last_y = 0.0f;
    bool first_ = true;

    float vx_robot = 0.0f;
    float vy_robot = 0.0f;
    float theta = 0.0f;
    u_int8_t sensor4=0;
    int16_t degree_rev =0 , pose_rev=0;

    rclcpp::Time last_time_;

    void readCAN()
    {
        int nbytes = read(socket_, &frame, sizeof(frame));
        if (nbytes <= 0) return;

        uint32_t id = frame.can_id & CAN_SFF_MASK;

        // ===== POSE FRAME =====
        if (id == 0x020)
        {
            int16_t robotx_raw = frame.data[0] | (frame.data[1] << 8);
            int16_t roboty_raw = frame.data[2] | (frame.data[3] << 8);
            int16_t th_raw     = frame.data[4] | (frame.data[5] << 8);
            int16_t laser_raw  = frame.data[6] | (frame.data[7] << 8);

            float robotx = robotx_raw / 1000.0f;
            float roboty = roboty_raw / 1000.0f;
            theta = th_raw / 1000.0f;
            float laser = laser_raw / 1000.0f;

            auto now = this->get_clock()->now();

            if (!first_)
            {
                double dt = (now - last_time_).seconds();

                if (dt < 0.001) dt = 0.001;
                if (dt > 0.05)  dt = 0.05;

                vx_robot = (robotx - last_x) / dt;
                vy_robot = (roboty - last_y) / dt;
            }
            else
            {
                first_ = false;
            }

            last_x = robotx;
            last_y = roboty;
            last_time_ = now;

            std_msgs::msg::Float32MultiArray msg;
            msg.data = {vx_robot, vy_robot, theta, laser};
            vel_pub_->publish(msg);

        }

        // ===== RESET FRAME =====
        if (id == 0x021)
        {
            uint8_t s[3] = {frame.data[0], frame.data[1], frame.data[2]};
            uint8_t rst_stm32 = frame.data[3];
            uint8_t btn = frame.data[4];
            std_msgs::msg::Float32MultiArray flag;        
            flag.data = {(float)s[0], (float)s[1], (float)s[2], (float)rst_stm32, (float)btn};
            stm32_pub->publish(flag);
        }
        if (frame.can_id == 0x104) // Check for the expected CAN ID
        {
                degree_rev = static_cast<int16_t>(
                    frame.data[0] | (frame.data[1] << 8)
                ) ;

                pose_rev = static_cast<int16_t>(
                    frame.data[2] | (frame.data[3] << 8)
                );
                sensor4 = frame.data[4];
                // RCLCPP_INFO(
                //     this->get_logger(),
                //     "Received CAN frame: degree_rev=%.4d, pose_rev=%.4d",
                //     degree_rev,
                //     pose_rev
                // );  
            robot_msg::msg::Motor motor_msg;
            motor_msg.degree_rev = static_cast<float>(degree_rev)/10; // Example: using degree_rev as degree
            motor_msg.pose_rev = static_cast<float>(pose_rev)/10000;   // Example: using pose_rev as pose
            motor_msg.sensor4 = sensor4;
            grip_pub->publish(motor_msg);
        }   
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CanRxNode>());
    rclcpp::shutdown();
    return 0;
}