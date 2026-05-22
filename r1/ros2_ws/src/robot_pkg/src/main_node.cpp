#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "robot_msg/msg/motor.hpp"
#include "robot_msg/msg/gamepad.hpp"


#include "ament_index_cpp/get_package_share_directory.hpp"

#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

#include <cmath>
#include <functional>
#include <string>

#include "robot_pkg/ekf_lib.hpp"
#include "robot_pkg/trajectory_loader.hpp"
#include "robot_pkg/pose_control_lib.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class BehaviorNode : public rclcpp::Node
{
public:
    BehaviorNode()
        : Node("behavior_node")
    {
        initParameters();
        initEKF();
        initSubscribers();
        initPublishers();
        initTimers();

        loadTrajectoryByState();

        RCLCPP_INFO(this->get_logger(), "Behavior Node Started");
    }

private:
    static constexpr float WALL = 3.2f;
    static constexpr float ROBOT_FRAME = 0.34f;

    float offset_x = 0.51f;
    float offset_y= 0.4f;
    float offset_theta= 0.0f;

    enum RetryState
    {
        RETRY_IDLE  = 0,
        RETRY_TASK1 = 1,
        RETRY_TASK2 = 2

    } retry_state_ = RETRY_IDLE;

    enum sizePlay
    {
        blue = 0,
        red

    } size_play_ = blue;

    enum robotmode{
        idle_mode = 0,
        auto_mode = 1,
        remote_mode = 2
    } robot_mode_ = auto_mode;
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

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr ekf_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr stm32_sub_;
    rclcpp::Subscription<robot_msg::msg::Gamepad>::SharedPtr recv_gp;
    rclcpp::Subscription<robot_msg::msg::Motor>::SharedPtr motor_rev;

    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr manual_ref_sub_;

    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_cmd_;
     //rclcpp::Publisher<robot_msg::msg::Txt>::SharedPtr vel_pub;
    rclcpp::Publisher<robot_msg::msg::Motor>::SharedPtr motor_cmd;


    rclcpp::TimerBase::SharedPtr main_timer_;
    rclcpp::TimerBase::SharedPtr ekf_timer_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr pubTimer_;
    rclcpp::TimerBase::SharedPtr motor_pub;
    rclcpp::TimerBase::SharedPtr vel_ramp;
    // rclcpp::TimerBase::SharedPtr timer2;
    // rclcpp::TimerBase::SharedPtr timer3;

    // Trajectory 
    std::vector<TrajectoryPoint> trajectory_;
    size_t traj_index_ = 0;
    std::string current_trajectory_ = "";
    bool traj_done = false;
    float traj_data[6] = {offset_x, offset_y, offset_theta, 0.0f, 0.0f, 0.0f};

    // enable/disable control
    bool manual_mode_ = false;
    bool pose_control_enabled_ = true;
    bool heading_control_enabled_ = false;

    // EKF
    EKF ekf_;

    float vx_enc = 0.0f;
    float vy_enc = 0.0f;
    float theta_imu = 0.0f;

    float laser_y_ = 0.0f;
    float y_new_ = 0.0f;

    float init_x_ = 0.0f;
    float init_y_ = 0.0f;
    float init_theta_ = 0.0f;

    rclcpp::Time last_time_;
    bool first_run_ = true;

    // stm32 status
    uint8_t stm32_s_[3] = {0, 0, 0};
    uint8_t stm32_rst_ = 0;
    uint8_t stm32_btn_ = 0;
    uint8_t auto_state_ = 0;
    uint8_t last_btn_state_ = 0;
    uint8_t last_reset_signal_ = 0;

    PoseControl controller_;

    float X_ = 0.0f;
    float Y_ = 0.0f;
    float theta_ = 0.0f;
    float theta_ref=0;

    float vx_ = 0.0f;
    float vy_ = 0.0f;
    float omega_ = 0.0f;

    int log_counter_ = 0;

    //menual_inital
    double vx= 0.0;
    double vy =0.0;
    double omega=0.0;
    double scaled =1.5;

    // Motor control
    double desire_deg ;
    double degree_rev ;

    double desire_pose=0.0f;
    double pose_rev=0.0f;

    // State machine
    int delay_tick=0, delay_tick2=0 ,delay_tick3=0;
    int button=0;
    int number=0;

    uint8_t sensor1 =0;
    uint8_t sensor2=0;
    uint8_t sensor3=0;

    double ramp_step = 0.00;
    double target_vx ;
    double target_vy;
    double target_omega;

    //end menual
    float sign_xref (float x_ref){
        if (size_play_ == blue)
        return x_ref;
        else
        return -x_ref;
    }
    
    void initParameters()
    {
        this->declare_parameter("init_x", offset_x);
        this->declare_parameter("init_y", offset_y);
        this->declare_parameter("init_theta", offset_theta);

        init_x_ = this->get_parameter("init_x").as_double();
        init_y_ = this->get_parameter("init_y").as_double();
        init_theta_ = this->get_parameter("init_theta").as_double();
    }

    void initEKF()
    {
        ekf_ = EKF();

        ekf_.setState(init_x_, init_y_, init_theta_);
    }

    void initSubscribers()
    {
        ekf_sub_ =
            this->create_subscription<std_msgs::msg::Float32MultiArray>("/cmd_vel_imu", 10,
                std::bind(&BehaviorNode::ekfCallback, this, std::placeholders::_1));

        stm32_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/stm32_status", 10,
                std::bind(&BehaviorNode::stm32Callback, this, std::placeholders::_1));

        manual_ref_sub_ = this->create_subscription<geometry_msgs::msg::Pose2D>("/manual_ref_pose", 10,
                std::bind(&BehaviorNode::manualRefCallback, this,std::placeholders::_1));
        
        //menual
        // Gamepad input
        recv_gp = this->create_subscription<robot_msg::msg::Gamepad>(
            "/GP",
            10,
            std::bind(&BehaviorNode::msg_callback, this, _1)
        );

        // Motor feedback
        motor_rev = this->create_subscription<robot_msg::msg::Motor>(
            "/motor_rev",
            10,
            std::bind(&BehaviorNode::motor_callback, this, _1)
        );
        //end menual
    }

    void initPublishers()
    {
        state_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/state_info", 10);

        pub_cmd_ = this->create_publisher<geometry_msgs::msg::Vector3>("/cmd_vel_custom", 10);
        
        // menual
        // vel_pub = this->create_publisher<robot_msg::msg::Txt>(
        //     "/vel",
        //     10
        // );

        // Motor command
        motor_cmd = this->create_publisher<robot_msg::msg::Motor>("/motor_cmd", 10);
        //end menual
    }

    void initTimers()
    {
        main_timer_ = this->create_wall_timer(std::chrono::milliseconds(5),
                std::bind(&BehaviorNode::behaviorLoop,this));
 
        ekf_timer_ = this->create_wall_timer(std::chrono::milliseconds(5),
                std::bind(&BehaviorNode::ekfLoop, this));

        control_timer_ = this->create_wall_timer(std::chrono::milliseconds(5),
                std::bind(&BehaviorNode::controlLoop,this));

        pubTimer_ = this->create_wall_timer(std::chrono::milliseconds(1),
                std::bind(&BehaviorNode::publoop,this));
        
        // // //menual

        motor_pub = this->create_wall_timer(5ms, std::bind(&BehaviorNode::publish_motor_cmd, this));
        vel_ramp = this->create_wall_timer(5ms, std::bind(&BehaviorNode::vel_ramp_fun, this));
        // // //end menual

    }

    void loadTrajectoryByState()
    {
        std::string package_path = ament_index_cpp::get_package_share_directory("robot_pkg");

        std::string filename;
        if (stm32_rst_ == 1){
            switch (retry_state_)
            {
                case RETRY_TASK1:
                    filename = package_path + "/trajectory_file/task1.txt";
                    traj_done = false;

                    break;

                case RETRY_TASK2:
                    filename = package_path +"/trajectory_file/retry_task2.txt";
                    traj_done = false;

                    break;

                default:
                    break;
            }
        }

        if (auto_state_ == 0 && retry_state_ == RETRY_IDLE)
        {
            traj_data[0] = sign_xref(offset_x);
            traj_data[1] = offset_y;
            traj_data[2] = offset_theta;

            traj_data[3] = 0.0f;
            traj_data[4] = 0.0f;
            traj_data[5] = 0.0f;

            traj_done = true;

            return;
        }

        else if (auto_state_ == 1)
        {
            filename = package_path + "/trajectory_file/task1.txt";

            traj_done = false;
        }

        else if (auto_state_ == 2)
        {
            filename = package_path +"/trajectory_file/task2.txt";

            traj_done = false;
        }

        if (filename.empty())
            return;

        if (filename == current_trajectory_)
            return;

        current_trajectory_ = filename;

        trajectory_ = TrajectoryLoader::load(filename);

        traj_index_ = 0;

        RCLCPP_WARN(this->get_logger(),"Loaded trajectory: %s | points: %ld",
            filename.c_str(), trajectory_.size());
    }

    void runTrajectory()
    {
        if (manual_mode_)
            return;

        if (trajectory_.empty())
            return;

        if (traj_done)
            return;

        auto &p = trajectory_[traj_index_];

        traj_data[0] = sign_xref(p.x);
        traj_data[1] = p.y;
        traj_data[2] = p.theta;

        traj_data[3] = p.vx;
        traj_data[4] = p.vy;
        traj_data[5] = p.omega;

        traj_index_++;

        if (traj_index_ >= trajectory_.size())
        {
            traj_done = true;

            RCLCPP_WARN(this->get_logger(), "Trajectory Finished");
        }
    }

    void ekfCallback(
        const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 4)
            return;

        vx_enc = msg->data[0];
        vy_enc = msg->data[1];

        theta_imu = msg->data[2];

        laser_y_ = msg->data[3];
    }

    void stm32Callback(
        const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 5)
            return;

        stm32_s_[0] = static_cast<uint8_t>(msg->data[0]);

        stm32_s_[1] = static_cast<uint8_t>(msg->data[1]);

        stm32_s_[2] = static_cast<uint8_t>(msg->data[2]);

        stm32_rst_ = static_cast<uint8_t>(msg->data[3]);

        stm32_btn_ = static_cast<uint8_t>(msg->data[4]);
    }

    void manualRefCallback(
        const geometry_msgs::msg::Pose2D::SharedPtr msg)
    {
        manual_mode_ = true;

        float x_ref = sign_xref(msg->x);
        traj_data[0] = x_ref;
        traj_data[1] = msg->y;
        traj_data[2] = msg->theta;
        // traj_data[2] = yaw_deg * M_PI / 180.0f;

        traj_data[3] = 0.0f;
        traj_data[4] = 0.0f;
        traj_data[5] = 0.0f;
    }

    void behaviorLoop()
    {
        choose_mode();
        size_play_ = (stm32_s_[2] == 0)? blue: red;
        if (robot_mode_ == idle_mode){
            pose_control_enabled_ = true;
            heading_control_enabled_ = false;
            traj_data[0] = sign_xref(offset_x);
            traj_data[1] = offset_y;
            traj_data[2] = offset_theta;

            traj_data[3] = 0.0f;
            traj_data[4] = 0.0f;
            traj_data[5] = 0.0f;
        }
        else if (robot_mode_ == auto_mode){
            pose_control_enabled_ = true;
            heading_control_enabled_ = false;
            robot_auto();
            RCLCPP_INFO(this->get_logger(),"x_ref: %.3f, y_ref: %.3f", traj_data[0], traj_data[1]);
        }
         else if (robot_mode_ == remote_mode){
            pose_control_enabled_ = false;
            heading_control_enabled_ = true;
            menual(); 
        }
    }

    void handleButton()
    {
        if (last_btn_state_ == 0 && stm32_btn_ == 1)
        {
            manual_mode_ = false;

            auto_state_++;

            current_trajectory_.clear();

            loadTrajectoryByState();

            RCLCPP_INFO(this->get_logger(),"AUTO STATE: %d",auto_state_);
        }
        last_btn_state_ = stm32_btn_;
    }

    void handleReset()
    {
        if (stm32_rst_ == 1)
        {

            uint8_t retry_s = stm32_s_[0]<<0|stm32_s_[1]<<1;
            switch (retry_s)
            {
            case 0:
                retry_state_ = RETRY_IDLE;
                break;
            case 1:
                retry_state_ = RETRY_TASK1;
                break;
            case 2:
                retry_state_ = RETRY_TASK2;
                break;
            default:
                break;}
                manual_mode_ = false;

                auto_state_ = 0;

                current_trajectory_.clear();

                loadTrajectoryByState();

                RCLCPP_WARN(
                    this->get_logger(),
                    "RESET PRESSED");
        }
    }
    void resetEKF(const std::string &reason)
    {
        ekf_ = EKF();
        ekf_.setState(
            init_x_,
            init_y_,
            init_theta_);

        vx_enc = 0.0f;
        vy_enc = 0.0f;

        theta_imu = init_theta_;

        last_time_ = this->now();

        first_run_ = true;

        RCLCPP_WARN(
            this->get_logger(),
            "RESET: %s",
            reason.c_str());
    }

    void ekfLoop()
    {
        // ekf_= EKF();
        if (stm32_rst_ != last_reset_signal_)
        {
            size_play_ = (stm32_s_[2] == 0)? blue: red;
            init_x_ = sign_xref(offset_x);
            resetEKF("STM32 Reset");

            last_reset_signal_ = stm32_rst_;
        }

        rclcpp::Time now = this->now();

        if (first_run_)
        {
            last_time_ = now;

            first_run_ = false;

            return;
        }

        double dt =
            (now - last_time_).seconds();

        last_time_ = now;

        if (dt < 0.001)
            dt = 0.001;

        if (dt > 0.05)
            dt = 0.05;

        y_new_ =
            WALL - ROBOT_FRAME - laser_y_;

        ekf_.predict(
            vx_enc,
            vy_enc,
            theta_imu,
            static_cast<float>(dt));

        ekf_.update(theta_imu);

        // if (auto_state_ == 1 && traj_done== true)
        // {
        //     x_est[1] = y_new_;
        // }

        auto state = ekf_.getState();

        X_ = state[0];
        Y_ = state[1];
        // theta_ = state[2];

        if (++log_counter_ >= 200)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "EKF -> x: %.3f y: %.3f th: %.3f, laser: %.3f",
                X_,
                Y_,
                theta_imu, laser_y_   );

            log_counter_ = 0;
        }
    }

    void controlLoop()
    {
        rclcpp::Time now = this->now();

        float dt =
            (now - last_time_).seconds();

        last_time_ = now;

        if (dt < 0.001)
            dt = 0.005;

        if (dt > 0.05)
            dt = 0.05;
        if (pose_control_enabled_ == true)
        {
            heading_control_enabled_ = false;
            controller_.setReference(
                traj_data[0],
                traj_data[1],
                traj_data[2] );
        
        controller_.computeControl(X_, Y_, theta_imu, dt, traj_data[3], traj_data[4], traj_data[5], vx_, vy_, omega_);
        }

        if (heading_control_enabled_ == true)
        {
            pose_control_enabled_ = false;
            if (omega!=0){
                theta_ref = theta_imu;
                omega_ = omega;
            }
            else {
                omega_ = controller_.headingControl(theta_ref, theta_imu, dt);
            }
            
            // vx=v_remote;
            // vy=vy_remote;
            vx_= cosf(theta_imu)*vx + sinf(theta_imu)*vy;
            vy_= -sinf(theta_imu)*vx+ cosf(theta_imu)*vy;
        }

        
    }

    void publoop(){
                geometry_msgs::msg::Vector3 cmd;
        cmd.x = vx_ ;
        cmd.y = vy_ ;
        cmd.z = omega_;
        pub_cmd_->publish(cmd);
    }
    //menual 

    void menual(void){
        
        test_gripper();
        change_speed();
        other_comand();
    
        
    }
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
        target_vx = (msg->lx * scaled) / 100.0;
        target_vy = (msg->ly * scaled) / 100.0;
        target_omega = ((-msg->rx) * 2.5) / 100.0;

    }
    void vel_ramp_fun(void){
        ramp(vx, target_vx, ramp_step);
        ramp(vy, target_vy, ramp_step);
        ramp(omega, target_omega, ramp_step);
    }
    void choose_mode(void){
        if (button & X5_A ){
            robot_mode_ = remote_mode;
        }
        else if (button & X5_Y ){
            robot_mode_ =auto_mode;
        }
    }
    void test_gripper(void){
        switch (button)
        {
        case X5_RB:
            number =0;
            break;
        case X5_RT:
            number =1;
            break;
        case X5_LB:
            number = 2;
            break;
        case X5_CHIKEN:
            number =3;
            break;
        
        default:
            break;
        }

        if (number == 0){
            state_IDLE();
        }
        else if (number ==1){
            state_test();
            delay_tick =0;
            delay_tick3=0;
        }
        else if (number == 2){
            state_grip_staff();
            delay_tick2 =0;
            delay_tick3=0;
        }
        else if (number ==3){
            life_saving();
            delay_tick2 =0;
            delay_tick =0;
        }
        }
    void change_speed(void){
        if ((button & X5_LT) && (button & X5_A)){
            ramp_step = 0.01;
            scaled = 0.2;
        RCLCPP_INFO(this->get_logger(),"robot_slow");
        }
        else if((button & X5_LT )&& (button & X5_B)){
            ramp_step = 0.01;
            scaled = 3.0;
            RCLCPP_INFO(this->get_logger(),"robot_fast_mode");
        }}
    void other_comand(void){
        switch (button)
        {
            case X5_B:
                state_Move_forward();
                break;
            case X5_X:
                state_Move_backward();
                break;
            case X5_UP:
                degree_up_to_5();
                break;
            case X5_DOWN:
                degree_down_to_95();
                break;
            default:
                break;
        }

    }
    
    /* =====================================================
       Publishers
    ===================================================== */
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
        delay_tick2 =0;
        delay_tick3=0;
        sensor1 = 1;
        sensor2 = 0;
        sensor3 = 0;

    }
    void state_grip_staff()
    {
        desire_deg = 0;
        delay_tick++;
        // sensor1 = 1;

         if (delay_tick >= 200)
        {
            sensor1 = 0;
        }
        if (delay_tick >= 300)
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

        if (fabs(pose_rev - 0.197) < 0.01)
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
            if (delay_tick3 >= 300)
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
    void degree_up_to_5(void){
        desire_deg +=0.05;
        if (desire_deg>=5){
            desire_deg =5;
        }
    }
    void degree_down_to_95(void){
        desire_deg -=0.05;
        if (desire_deg<=-95){
            desire_deg =-95;
        }
    }

    void robot_auto(){
    handleReset();
    runTrajectory();
    handleButton();
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<BehaviorNode>());

    rclcpp::shutdown();

    return 0;
}