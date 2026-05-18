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
class CanRxNode : public rclcpp::Node
{
public:
    CanRxNode() : Node("can_rx_node"){
        canrx_setup();
        sensor1 = 0, sensor2 =0;
        degree_rev = 0, pose_rev = 0;
        grip_pub = this->create_publisher<robot_msg::msg::Motor>("/motor_rev", 10);
        timer = this->create_wall_timer(1ms, std::bind(&CanRxNode::read_can, this));
    }
    ~CanRxNode(){ close(socket_can_); }
private:
    int socket_can_;
    u_int8_t sensor1, sensor2;
    int16_t degree_rev, pose_rev;

    rclcpp::Publisher<robot_msg::msg::Motor>::SharedPtr grip_pub;
    rclcpp::TimerBase::SharedPtr timer;
    void canrx_setup(void){
        /* Create CAN socket */
        socket_can_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);

        if (socket_can_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "CAN socket creation failed");
            rclcpp::shutdown();
            return;
        }

        /* Select interface: can1 */
        struct ifreq ifr;
        std::strcpy(ifr.ifr_name, "can0");
        ioctl(socket_can_, SIOCGIFINDEX, &ifr);

        /* Bind socket */
        struct sockaddr_can addr;
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(socket_can_,
                 (struct sockaddr *)&addr,
                 sizeof(addr)) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "CAN bind failed");
            close(socket_can_);
            rclcpp::shutdown();
            return;
        }

    }
    void read_can(){
        struct can_frame frame;
        ssize_t nbytes = read(socket_can_, &frame, sizeof(frame));

        if (nbytes < static_cast<ssize_t>(sizeof(frame)))
        {
            RCLCPP_WARN(this->get_logger(), "Incomplete CAN frame");
            return;
        }
    
        if (nbytes < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "CAN read error");
            return;
        }

        // Process the received CAN frame
        if (frame.can_id == 0x104) // Check for the expected CAN ID
        {
                degree_rev = static_cast<int16_t>(
                    frame.data[0] | (frame.data[1] << 8)
                ) ;

                pose_rev = static_cast<int16_t>(
                    frame.data[2] | (frame.data[3] << 8)
                );

                RCLCPP_INFO(
                    this->get_logger(),
                    "Received CAN frame: degree_rev=%.4d, pose_rev=%.4d",
                    degree_rev,
                    pose_rev
                );  
            robot_msg::msg::Motor motor_msg;
            motor_msg.degree_rev = static_cast<float>(degree_rev)/10; // Example: using degree_rev as degree
            motor_msg.pose_rev = static_cast<float>(pose_rev)/10000;   // Example: using pose_rev as pose
            motor_msg.sensor1 = sensor1;
            motor_msg.sensor2 = sensor2;

            grip_pub->publish(motor_msg);
        }   

}
};
int main (int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CanRxNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}