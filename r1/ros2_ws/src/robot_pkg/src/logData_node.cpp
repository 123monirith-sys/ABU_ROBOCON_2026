#include "rclcpp/rclcpp.hpp"

#include <fstream>
#include <chrono>
#include <functional>
#include <ctime>
#include <sstream>
#include <filesystem>

#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

class LoggerNode : public rclcpp::Node
{
public:
    LoggerNode() : Node("logger_node")
    {
        // ===== Create logs folder =====
        std::filesystem::create_directories("logs");

        // ===== Create unique filename =====
        auto now = std::time(nullptr);
        auto tm = *std::localtime(&now);

        std::stringstream ss;

        ss << "robot_log_"
           << (tm.tm_year + 1900)
           << "_"
           << (tm.tm_mon + 1)
           << "_"
           << tm.tm_mday
           << "_"
           << tm.tm_hour
           << "_"
           << tm.tm_min
           << "_"
           << tm.tm_sec
           << ".csv";

        // save inside logs folder
        filename_ = "logs/" + ss.str();

        // ===== Open new file =====
        file_.open(filename_, std::ios::out);

        if (!file_.is_open())
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to open log file!");
            return;
        }

        // CSV header
        file_ << "time,x_ekf,y_ekf,theta_ekf,x_ref,y_ref,theta_ref,ex,ey,etheta" << std::endl;

        start_time_ = this->now();

        // ===== Subscriber =====
        logdata_sub_ =
            this->create_subscription<std_msgs::msg::Float32MultiArray>(
                "/log_data",
                10,
                std::bind(
                    &LoggerNode::logdataCallback,
                    this,
                    _1));

        // ===== Timer =====
        timer_ = this->create_wall_timer(
            10ms,
            std::bind(
                &LoggerNode::loop,
                this));

        RCLCPP_INFO(
            this->get_logger(),
            "Logging to file: %s",
            filename_.c_str());
    }

    ~LoggerNode()
    {
        if (file_.is_open())
        {
            file_.close();
        }
    }

private:
    // ===== Data =====
    float x_ekf = 0.0f;
    float y_ekf = 0.0f;
    float theta_imu = 0.0f;
    float x_ref= 0.0f;
    float y_ref= 0.0f;
    float theta_ref= 0.0f;
    float ex_ = 0.0f;
    float ey_ = 0.0f;
    float etheta_ = 0.0f;

    std::string filename_;

    std::ofstream file_;

    rclcpp::Time start_time_;

    // ===== ROS =====
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
        logdata_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    // ===== Callback =====
    void logdataCallback(
        const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 3)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Received log data with insufficient size");
            return;
        }

        x_ekf = msg->data[0];
        y_ekf = msg->data[1];
        theta_imu = msg->data[2];
        x_ref = msg->data[3];
        y_ref = msg->data[4];
        theta_ref = msg->data[5];
        ex_ = msg->data[6];
        ey_ = msg->data[7];
        etheta_ = msg->data[8]; 
    }

    // ===== Main Loop =====
    void loop()
    {
        double t =
            (this->now() - start_time_).seconds();

        // write CSV
        file_ << t << ","
              << x_ekf << ","
              << y_ekf << ","
              << theta_imu << ","
              << x_ref << ","
              << y_ref << ","
              << theta_ref << ","
              << ex_ << ","
              << ey_ << ","
              << etheta_
              << std::endl;

    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<LoggerNode>());

    rclcpp::shutdown();

    return 0;
}