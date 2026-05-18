
#include "rclcpp/rclcpp.hpp"
#include "robot_msg/msg/txt.hpp"
#include "robot_msg/msg/motor.hpp"

// Linux CAN
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <cstring>

using std::placeholders::_1;
using namespace std::chrono_literals;
class CanTxNode : public rclcpp::Node
{
public:
    CanTxNode() : Node("can_tx_node"){
        sensor1 = 0, sensor2 =0, sensor3=0 ;
        desire_deg = 0, desire_pose = 0;
        can_setup();
        vel_sub = this->create_subscription<robot_msg::msg::Txt>("/vel",10, std::bind(&CanTxNode::vel_callback, this,_1));
        cmd_motor_sub = this->create_subscription<robot_msg::msg::Motor>("/motor_cmd",10, std::bind(&CanTxNode::motor_callback, this,_1));

        

    }
    ~CanTxNode(){ close(can_socket); }
private:
    int can_socket;
    int16_t vx, vy, omega,desire_deg, desire_pose;
    u_int8_t sensor1, sensor2, sensor3; // sensor3 is for padding

    rclcpp::Subscription<robot_msg::msg::Txt>::SharedPtr vel_sub;
    rclcpp::Subscription<robot_msg::msg::Motor>::SharedPtr cmd_motor_sub;
    void can_setup(void){
        struct sockaddr_can addr;
        struct ifreq ifr;
        can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if(can_socket < 0){
            RCLCPP_ERROR(this->get_logger(), "CAN socket create failed");
            return;
        }
        std::strcpy(ifr.ifr_name, "can0");
        ioctl(can_socket, SIOCGIFINDEX, &ifr);
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if(bind(can_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0){
            RCLCPP_ERROR(this->get_logger(), "CAN bind failed");
            return; 
        }
    }

    void vel_callback( const robot_msg::msg::Txt::SharedPtr msg){
        vx = static_cast<int16_t>(msg->vx * 1000);
        vy = static_cast<int16_t>(msg->vy * 1000);
        omega = static_cast<int16_t>(msg->omega * 1000);

        struct can_frame frame;
        frame.can_id = 0x005;
        frame.can_dlc = 6;
        std::memcpy(&frame.data[0], &vx, 2);
        std::memcpy(&frame.data[2], &vy, 2);
        std::memcpy(&frame.data[4], &omega, 2);
        if(write(can_socket, &frame, sizeof(frame)) < 0){
            RCLCPP_ERROR(this->get_logger(), "Failed to send velocity CAN frame");  

        }
    }
    void motor_callback(const robot_msg::msg::Motor::SharedPtr msg){
        desire_deg = static_cast<int16_t>(msg->degree);   
        desire_pose = static_cast<int16_t>(msg->pose*10000);
        sensor1 = static_cast<u_int8_t>(msg->sensor1);
        sensor2 = static_cast<u_int8_t>(msg->sensor2);
        sensor3 = static_cast<u_int8_t>(msg->sensor3);
    
        struct can_frame flame2;
        flame2.can_id = 0x103;
        flame2.can_dlc = 8;
        std::memcpy(&flame2.data[0], &desire_deg, 2);
        std::memcpy(&flame2.data[2], &desire_pose, 2);
        // std::memcpy(&flame2.data[4], &sensor1, 1);
        // std::memcpy(&flame2.data[5], &sensor2, 1);
        // std::memcpy(&flame2.data[6], &sensor3, 1); // Padding bytes
            flame2.data[4] = sensor1;
            flame2.data[5] = sensor2;
            flame2.data[6] = sensor3; // Padding byte
        RCLCPP_INFO(this->get_logger(), "Sending motor command: degree=%d, pose=%d, sensor1=%d, sensor2=%d ,sensor3=%d", desire_deg, desire_pose, sensor1, sensor2, sensor3    );

        if(write(can_socket, &flame2, sizeof(flame2)) < 0){
            RCLCPP_ERROR(this->get_logger(), "Failed to send motor CAN frame");
        }
    }
};
int main (int argc, char * argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CanTxNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}