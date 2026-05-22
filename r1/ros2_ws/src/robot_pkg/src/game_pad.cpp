#include <rclcpp/rclcpp.hpp>
#include <robot_msg/msg/gamepad.hpp>
#include <robot_msg/msg/txt.hpp>

#include <nlohmann/json.hpp>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>

#include <string>
#include <cstring>

using json = nlohmann::json;
using std::placeholders::_1;

#define PC_PORT 8888

// =============================
// Button Bitmasks
// =============================
#define X5_A       (1 << 0)
#define X5_B       (1 << 1)
#define X5_X       (1 << 2)
#define X5_Y       (1 << 3)
#define X5_SHERE   (1 << 4)
#define X5_CHIKEN  (1 << 5)
#define X5_MENU    (1 << 6)
#define X5_L3      (1 << 7)
#define X5_R3      (1 << 8)
#define X5_LB      (1 << 9)
#define X5_RB      (1 << 10)
#define X5_UP      (1 << 11)
#define X5_DOWN    (1 << 12)
#define X5_LEFT    (1 << 13)
#define X5_RIGHT   (1 << 14)
#define X5_LT      (1 << 15)
#define X5_RT      (1 << 16)

class X5LiteNode : public rclcpp::Node
{
public:
    X5LiteNode() : Node("game_pad_node")
    {
        cmd_pub_ = this->create_publisher<robot_msg::msg::Gamepad>("/GP", 10);

        traject_end_sub_ = this->create_subscription<robot_msg::msg::Txt>(
            "/data_txt",
            10,
            std::bind(&X5LiteNode::trajectory_end_callback, this, _1)
        );

        // =============================
        // UDP Socket Setup
        // =============================
        sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);

        if (sock_fd_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Socket creation failed");
            return;
        }

        sockaddr_in server_addr{};
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = INADDR_ANY;
        server_addr.sin_port = htons(PC_PORT);

        if (bind(sock_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Bind failed");
            close(sock_fd_);
            return;
        }

        // Non-blocking mode
        fcntl(sock_fd_, F_SETFL, O_NONBLOCK);

        RCLCPP_INFO(this->get_logger(), "Listening on UDP %d", PC_PORT);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5),
            std::bind(&X5LiteNode::udp_callback, this)
        );

        end_value_ = 0;
    }

    ~X5LiteNode()
    {
        if (sock_fd_ > 0)
        {
            close(sock_fd_);
        }
    }

private:
    // =============================
    // Deadzone
    // =============================
    float deadzone(float v)
    {
        if (std::abs(v) < 6.0)
            return 0.0;
        return v;
    }

    // =============================
    // Subscriber Callback
    // =============================
    void trajectory_end_callback(const robot_msg::msg::Txt::SharedPtr msg)
    {
        end_value_ = msg->done;
    }

    // =============================
    // UDP Receive Callback
    // =============================
    void udp_callback()
    {
        char buffer[1024] = {0};
        sockaddr_in client_addr{};
        socklen_t addr_len = sizeof(client_addr);

        int len = recvfrom(
            sock_fd_,
            buffer,
            sizeof(buffer) - 1,
            0,
            (struct sockaddr*)&client_addr,
            &addr_len
        );

        if (len <= 0)
        {
            return;
        }

        try
        {
            buffer[len] = '\0';
            json msg = json::parse(buffer);

            float lx = deadzone(msg.value("lx", 0.0));
            float ly = deadzone(msg.value("ly", 0.0));
            float rx = deadzone(msg.value("rx", 0.0));
            float ry = deadzone(msg.value("ry", 0.0));

            int btn = msg.value("btn", 0);
            int btnm = msg.value("btnM", 0);

            // =============================
            // Publish ROS2 Message
            // =============================
            auto gp = robot_msg::msg::Gamepad();

            gp.lx = lx;
            gp.ly = ly;
            gp.rx = rx;
            gp.ry = ry;
            gp.btn = btn;
            gp.btnm = btnm;

            // gp.a = (btn & X5_A) != 0;
            // gp.b = (btn & X5_B) != 0;
            // gp.x = (btn & X5_X) != 0;
            // gp.y = (btn & X5_Y) != 0;
            // gp.shere = (btn & X5_SHERE) != 0;
            // gp.chiken = (btn & X5_CHIKEN) != 0;
            // gp.menu = (btn & X5_MENU) != 0;
            // gp.l3 = (btn & X5_L3) != 0;
            // gp.r3 = (btn & X5_R3) != 0;
            // gp.lb = (btn & X5_LB) != 0;
            // gp.rb = (btn & X5_RB) != 0;
            // gp.up = (btn & X5_UP) != 0;
            // gp.down = (btn & X5_DOWN) != 0;
            // gp.left = (btn & X5_LEFT) != 0;
            // gp.right = (btn & X5_RIGHT) != 0;
            // gp.lt = (btn & X5_LT) != 0;
            // gp.rt = (btn & X5_RT) != 0;
            

            cmd_pub_->publish(gp);

            // =============================
            // Send End Back
            // =============================
            json response;
            response["End"] = end_value_;

            std::string response_str = response.dump();

            sendto(
                sock_fd_,
                response_str.c_str(),
                response_str.size(),
                0,
                (struct sockaddr*)&client_addr,
                addr_len
            );

            RCLCPP_INFO(
                this->get_logger(),
                "End: %d",
                end_value_
            );
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "JSON parse error: %s",
                e.what()
            );
        }
    }

private:
    rclcpp::Publisher<robot_msg::msg::Gamepad>::SharedPtr cmd_pub_;
    rclcpp::Subscription<robot_msg::msg::Txt>::SharedPtr traject_end_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    int sock_fd_;
    int end_value_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<X5LiteNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}