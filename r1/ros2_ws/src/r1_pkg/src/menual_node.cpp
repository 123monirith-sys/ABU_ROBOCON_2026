#include "menual_node.hpp"

MenualNode::MenualNode()
: Node("menual_node")
{
    vx = 0.0;
    vy = 0.0;
    omega = 0.0;
    scaled = 3.0;

    desire_deg = 0.0;
    degree_rev = 0.0;

    desire_pose = 0.0;
    pose_rev = 0.0;

    sensor1 = 0;
    sensor2 = 0;

    delay_tick = 0;
    button = 0;
    number = 0;

    recv_gp = this->create_subscription<robot_msg::msg::Gamepad>(
        "/GP",
        10,
        std::bind(&MenualNode::msg_callback, this, _1)
    );

    motor_rev = this->create_subscription<robot_msg::msg::Motor>(
        "/motor_rev",
        10,
        std::bind(&MenualNode::motor_callback, this, _1)
    );

    vel_pub = this->create_publisher<robot_msg::msg::Txt>(
        "/vel",
        10
    );

    motor_cmd = this->create_publisher<robot_msg::msg::Motor>(
        "/motor_cmd",
        10
    );

    timer = this->create_wall_timer(
        std::chrono::milliseconds(5),
        std::bind(&MenualNode::publish_vel, this)
    );

    timer2 = this->create_wall_timer(
        std::chrono::milliseconds(5),
        std::bind(&MenualNode::test_gripper, this)
    );

    timer3 = this->create_wall_timer(
        std::chrono::milliseconds(5),
        std::bind(&MenualNode::publish_motor_cmd, this)
    );

    RCLCPP_INFO(this->get_logger(), "Menual Node Started");
}

void MenualNode::velocity(
    const robot_msg::msg::Gamepad::SharedPtr msg)
{
    vx = (msg->lx * scaled) / 100.0;
    vy = (msg->ly * scaled) / 100.0;
    omega = (msg->rx * scaled) / 100.0;
}