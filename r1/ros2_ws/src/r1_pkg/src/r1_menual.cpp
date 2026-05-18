#include <memory>
#include <chrono>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "robot_msg/msg/gamepad.hpp"
#include "robot_msg/msg/txt.hpp"
#include "robot_msg/msg/motor.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

/* =====================================================
   Button Bitmask Definitions
===================================================== */
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

/* =====================================================
   Main Node
===================================================== */
class MenualNode : public rclcpp::Node
{
public:
    MenualNode() : Node("menual_node")
    {
        /* ===============================
           Initial Values
        =============================== */
        vx = 0.0;
        vy = 0.0;
        omega = 0.0;
        scaled = 1.5;

        desire_deg = 0.0;
        degree_rev = 0.0;

        desire_pose = 0.0;
        pose_rev = 0.0;

        sensor1 = 0;
        sensor2 = 0, sensor3 = 0;

        delay_tick = 0, delay_tick2 = 0 ,delay_tick3 = 0;
        button = 0;
        number =0;

        /* ===============================
           Subscribers
        =============================== */

        // Gamepad input
        recv_gp = this->create_subscription<robot_msg::msg::Gamepad>(
            "/GP",
            10,
            std::bind(&MenualNode::msg_callback, this, _1)
        );

        // Motor feedback
        motor_rev = this->create_subscription<robot_msg::msg::Motor>(
            "/motor_rev",
            10,
            std::bind(&MenualNode::motor_callback, this, _1)
        );

        /* ===============================
           Publishers
        =============================== */

        // Robot velocity
        vel_pub = this->create_publisher<robot_msg::msg::Txt>(
            "/vel",
            10
        );

        // Motor command
        motor_cmd = this->create_publisher<robot_msg::msg::Motor>(
            "/motor_cmd",
            10
        );

        /* ===============================
           Timers (5ms)
        =============================== */

        timer = this->create_wall_timer(
            5ms,
            std::bind(&MenualNode::publish_vel, this)
        );

        timer2 = this->create_wall_timer(
            5ms,
            std::bind(&MenualNode::test_gripper, this)
        );

        timer3 = this->create_wall_timer(
            5ms,
            std::bind(&MenualNode::publish_motor_cmd, this)
        );

        RCLCPP_INFO(this->get_logger(), "Menual Node Started");
    }

private:

    /* =====================================================
       Subscriber Callbacks
    ===================================================== */

    void msg_callback(const robot_msg::msg::Gamepad::SharedPtr msg)
    {
        button = msg->btn;
        velocity(msg);
       
        
    }

    void motor_callback(const robot_msg::msg::Motor::SharedPtr msg)
    {
        degree_rev = msg->degree_rev;
        pose_rev = msg->pose_rev;
    }

    /* =====================================================
       Velocity Calculation
    ===================================================== */
    void ramp(double &value, double target, double step)
    {
        if (value < target)
        {
            value += step;
            if (value > target)
                value = target;
        }
        else if (value > target)
        {
            value -= step;
            if (value < target)
                value = target;
        }
    }
    void velocity(const robot_msg::msg::Gamepad::SharedPtr msg)
    {
        double target_vx = (msg->lx * scaled) / 100.0;
        double target_vy = (msg->ly * scaled) / 100.0;
        double target_omega = ((-msg->rx) * scaled) / 100.0;

        

        ramp(vx, target_vx, ramp_step);
        ramp(vy, target_vy, ramp_step);
        ramp(omega, target_omega, ramp_step);
    }
    /* =====================================================
       State Machine Trigger
    ===================================================== */

    void test_gripper()
    {
        switch (button)
        {
            case X5_A:
                number = 1;
                break;
            case X5_B:
                number = 3;
                state_Move_forward();
                break;
            case X5_X:
                number = 4;
                state_Move_backward();
                break;
            case X5_Y:
                number = 2;
                break;
            case X5_RT:
                number = 5;
                break;
            case X5_RB:
                number = 0;
                break;
            case X5_LB:
                number = 6;
                break;
            case X5_LT:
                number = 7;
                break;
            case X5_SHERE:
                number = 8;
                break;
            default:
                break;
        
        }
         RCLCPP_INFO(this->get_logger(), "number: %d", number);
        if (number == 1)
                {
                    RCLCPP_INFO(this->get_logger(), "Gripping staff...");
                    state_grip_staff();
                    delay_tick2=0;
                    delay_tick3=0;
                    

                }
        else if (number == 2)
                {
                    RCLCPP_INFO(this->get_logger(), "Standing...");
                    state_stand();
                    delay_tick =0;
                    delay_tick3 =0;

                }
        else if (number == 0)
                {
                    RCLCPP_INFO(this->get_logger(), "Idle...");
                    state_IDLE();
                    delay_tick =0;
                    delay_tick2=0;
                    delay_tick3=0;
                }
        else if (number == 5){
                    RCLCPP_INFO(this->get_logger(), "Testing...");
                    state_test();
                    delay_tick =0;
                    delay_tick2=0;
                    delay_tick3=0;
        
        }
        else if (number == 6){
                    RCLCPP_INFO(this->get_logger(), "Life Saving...");
                    life_saving();
                    delay_tick =0;
                    delay_tick2=0;
        
        }
        else if (number == 7){
            ramp_step = 0.01;
            scaled = 0.2;
        }
       else if (number == 8){
            ramp_step = 0.1;
            scaled = 1.5;
        }
    }

    /* =====================================================
       Publishers
    ===================================================== */

    void publish_vel()
    {
        robot_msg::msg::Txt data_msg;

        data_msg.vx = vx;
        data_msg.vy = vy;
        data_msg.omega = omega;

        vel_pub->publish(data_msg);
    }

    void publish_motor_cmd()
    {
        robot_msg::msg::Motor motor_msg;

        motor_msg.degree = desire_deg;
        motor_msg.pose = desire_pose;
        motor_msg.sensor1 = sensor1;
        motor_msg.sensor2 = sensor2;
        motor_msg.sensor3 = sensor3;
        motor_cmd->publish(motor_msg);
    }

    /* =====================================================
       State Machine Functions
    ===================================================== */
    void state_IDLE(){
        desire_deg = 0;
        desire_pose = 0;
        delay_tick=0;
        sensor1 = 1;
        sensor2 = 0;
        sensor3 = 0;

    }
    void state_grip_staff()
    {
        desire_deg = 0;
        delay_tick++;
        // sensor1 = 1;

         if (delay_tick >= 50)
        {
            sensor1 = 0;
        }
        if (delay_tick >= 100)
        {   sensor3 = 0;
            desire_deg = -90;
            
            
        }
         if (degree_rev <= -89)
        {   
            sensor2 = 1;
        }
        else{
            sensor2 = 0;
        }

    }
    void state_test(){
        
        desire_pose = 0.197;

        if (pose_rev >= 0.197)
        {
            sensor3 = 1;
        }


    }
    void state_stand()
    {   sensor2 = 0;
        desire_deg = 0;
        sensor1 = 0;
        delay_tick2++;

        if (degree_rev != 0){
            sensor3 = 0;
        }
    }
    void life_saving(void){
        sensor2 = 0;
        desire_deg = 0;
        sensor1 = 0;
        delay_tick3++;

         if (degree_rev == 0)
        {
            if (delay_tick3 >= 100)
            {
                sensor3 = 0;
            }
            
        }
    }
    void state_Move_forward()
        {
            desire_pose += 0.0005;

            if (desire_pose > 0.27)
            {
                desire_pose = 0.27;
            }
        }
    void state_Move_backward()
        {
            desire_pose -= 0.0005;

            if (desire_pose < 0.0)
            {
                desire_pose = 0.0;
            }
        }


    /* =====================================================
       Variables
    ===================================================== */

    // Velocity
    double vx;
    double vy;
    double omega;
    double scaled;

    // Motor control
    double desire_deg;
    double degree_rev;

    double desire_pose;
    double pose_rev;

    // State machine
    int delay_tick, delay_tick2 ,delay_tick3;
    int button;
    int number;

    uint8_t sensor1;
    uint8_t sensor2;
    uint8_t sensor3;

    double ramp_step = 0.00;
    /* =====================================================
       ROS2 Interfaces
    ===================================================== */

    rclcpp::Subscription<robot_msg::msg::Gamepad>::SharedPtr recv_gp;
    rclcpp::Subscription<robot_msg::msg::Motor>::SharedPtr motor_rev;

    rclcpp::Publisher<robot_msg::msg::Txt>::SharedPtr vel_pub;
    rclcpp::Publisher<robot_msg::msg::Motor>::SharedPtr motor_cmd;

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::TimerBase::SharedPtr timer2;
    rclcpp::TimerBase::SharedPtr timer3;
};

/* =====================================================
   Main
===================================================== */

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MenualNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}