#ifndef MENUAL_NODE_HPP
#define MENUAL_NODE_HPP

#include <memory>
#include <chrono>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "robot_msg/msg/gamepad.hpp"
#include "robot_msg/msg/txt.hpp"
#include "robot_msg/msg/motor.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

enum ButtonState
{
    X5_A      = (1 << 0),
    X5_B      = (1 << 1),
    X5_X      = (1 << 2),
    X5_Y      = (1 << 3),
    X5_SHERE  = (1 << 4),
    X5_CHIKEN = (1 << 5),
    X5_MENU   = (1 << 6),
    X5_L3     = (1 << 7),
    X5_R3     = (1 << 8),
    X5_LB     = (1 << 9),
    X5_RB     = (1 << 10),
    X5_UP     = (1 << 11),
    X5_DOWN   = (1 << 12),
    X5_LEFT   = (1 << 13),
    X5_RIGHT  = (1 << 14),
    X5_LT     = (1 << 15),
    X5_RT     = (1 << 16)
};

class MenualNode : public rclcpp::Node
{
public:
    MenualNode();

private:
    /* callbacks */
    void msg_callback(const robot_msg::msg::Gamepad::SharedPtr msg);
    void motor_callback(const robot_msg::msg::Motor::SharedPtr msg);

    /* logic */
    void velocity(const robot_msg::msg::Gamepad::SharedPtr msg);
    void test_gripper();

    /* publishers */
    void publish_vel();
    void publish_motor_cmd();

    /* states */
    void state_IDLE();
    void state_grip_staff();
    void state_Move_forward();
    void state_Move_backward();
    void state_stand();

    /* variables */
    double vx, vy, omega, scaled;
    double desire_deg, degree_rev;
    double desire_pose, pose_rev;

    int delay_tick;
    int button;
    int number;

    uint8_t sensor1;
    uint8_t sensor2;

    /* ROS2 */
    rclcpp::Subscription<robot_msg::msg::Gamepad>::SharedPtr recv_gp;
    rclcpp::Subscription<robot_msg::msg::Motor>::SharedPtr motor_rev;

    rclcpp::Publisher<robot_msg::msg::Txt>::SharedPtr vel_pub;
    rclcpp::Publisher<robot_msg::msg::Motor>::SharedPtr motor_cmd;

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::TimerBase::SharedPtr timer2;
    rclcpp::TimerBase::SharedPtr timer3;
};

#endif