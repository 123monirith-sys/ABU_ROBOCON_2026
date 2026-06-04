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

// lib
#include "robot_pkg/ekf_lib.hpp"
#include "robot_pkg/trajectory_loader.hpp"
#include "robot_pkg/pose_control_lib.hpp"
#include "robot_pkg/laserToworld.hpp"
#include "robot_pkg/buttongui_lib.hpp"
#include "robot_pkg/gam_pad_lib.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;
ButtonguiLib gui_;

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
        size_play_ = (stm32_s_[2] == 0)? blue: red;
        size_play_problem();

        RCLCPP_INFO(this->get_logger(), "Behavior Node Started");
    }

private:
    void button_gui(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
    void can_feedback_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    static constexpr float WALL_Y = 3.2f;
    static constexpr float WALL_X = 6.0f;
    static constexpr float ROBOT_FRAME_Y = 0.35f;
    static constexpr float ROBOT_FRAME_X = 0.35f;
    static constexpr float offsetx_red = -0.38f;
    static constexpr float offsetx_blue = 0.61f;
    float offset_x = 0.0f;
    float offset_y= 0.38+0.13f;
    float offset_theta= 1.5708f; //90 degree

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
    // enum ButtonState
    // {
    //     X5_A      = (1 << 0),
    //     X5_B      = (1 << 1),
    //     X5_X      = (1 << 2),
    //     X5_Y      = (1 << 3),
    //     X5_SHERE  = (1 << 4),
    //     X5_CHIKEN = (1 << 5),
    //     X5_MENU   = (1 << 6),
    //     X5_L3     = (1 << 7),
    //     X5_R3     = (1 << 8),
    //     X5_LB     = (1 << 9),
    //     X5_RB     = (1 << 10),
    //     X5_UP     = (1 << 11),
    //     X5_DOWN   = (1 << 12),
    //     X5_LEFT   = (1 << 13),
    //     X5_RIGHT  = (1 << 14),
    //     X5_LT     = (1 << 15),
    //     X5_RT     = (1 << 16)
    // };

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr ekf_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr stm32_sub_;
    rclcpp::Subscription<robot_msg::msg::Gamepad>::SharedPtr recv_gp;
    rclcpp::Subscription<robot_msg::msg::Motor>::SharedPtr motor_rev;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr manual_ref_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr recv_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr can_feedback_sub_;
    
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_cmd_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr range_finder_sub_;
     //rclcpp::Publisher<robot_msg::msg::Txt>::SharedPtr vel_pub;
    rclcpp::Publisher<robot_msg::msg::Motor>::SharedPtr motor_cmd;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr xyt_pub;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr logdata_pub;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr ttt_pub_;

    rclcpp::TimerBase::SharedPtr main_timer_;
    rclcpp::TimerBase::SharedPtr ekf_timer_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr logdata_timer_;
    // rclcpp::TimerBase::SharedPtr pubTimer_;
    rclcpp::TimerBase::SharedPtr motor_pub;
    //rclcpp::TimerBase::SharedPtr vel_ramp;
    rclcpp::TimerBase::SharedPtr send_xyt_gui;

    //
    bool start_robot = true; // dak true sin, pel mean button do tov false vinh 

    // Trajectory 
    std::vector<TrajectoryPoint> trajectory_;
    size_t traj_index_ = 0;
    std::string current_trajectory_ = "";
    bool traj_done = false;
    float traj_data[6] = {offset_x, offset_y, offset_theta, 0.0f, 0.0f, 0.0f};
    std::string task_1 = "";
    std::string task_2 = "";
    std::string retry_task_2 = "";

    // enable/disable control
    bool manual_mode_ = false;
    bool pose_control_enabled_ = true;
    bool heading_control_enabled_ = false;

    // EKF
    EKF ekf_;

    float x_enc = 0.0f;
    float y_enc = 0.0f;
    float prev_x_enc = 0.0f;
    float prev_y_enc = 0.0f;
    bool have_enc_pose_ = false;
    bool have_prev_enc_pose_ = false;
    rclcpp::Time enc_pose_time_;
    rclcpp::Time prev_enc_pose_time_;

    float theta_imu = 0.0f;

    float y_new_ = 0.0f;
    float x_new_ = 0.0f;

    float init_x_ = 0.0f;
    float init_y_ = 0.0f;
    float init_theta_ = 0.0f;

    bool correct_ekf = false;

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
    float theta_ref= offset_theta;

    float vx_ = 0.0f;
    float vy_ = 0.0f;
    float omega_ = 0.0f;

    int log_counter_ = 0;

    // range_finder
    float range_finder[3] = {0.0f, 0.0f ,0.0f};


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
    int delay_tick0=0,delay_tick=0, delay_tick2=0 ,delay_tick3=0;
    int button=0;
    int number      = -1;   // ← -1 not 0
    int last_number = -1;   // ← member, not local
    int ttt_number = 0;


    uint8_t sensor1 =0;
    uint8_t sensor2=0;
    uint8_t sensor3=0;

    double ramp_step = 0.00;
    double target_vx ;
    double target_vy;
    double target_omega;


// TIC TAC TOE CAN TARGET

    float can_forward = 0.0f;
    float can_grip1  = 45.0f;
    float can_lift   = 0.0f;
    float can_grip2  = 0.0f;

// feedback
    float fb_forward = 0.0f;
    float fb_grip1   = 0.0f;
    float fb_lift    = 0.0f;
    float fb_grip2   = 0.0f;
    int rt_step = 0;
    int rb_step = 0;
    bool grip_closed = false;
    int lt_step = 0;
    int lb_step = 0;

    rclcpp::Time step_time;

    bool rt_wait = false;
    bool rb_wait = false;
    bool lt_wait = false;
    bool lb_wait = false;

    //end menual
    
    void initParameters()
    {
        this->declare_parameter("init_x", offset_x);
        this->declare_parameter("init_y", offset_y);
        this->declare_parameter("init_theta", offset_theta);

        init_x_ = this->get_parameter("init_x").as_double();
        init_y_ = this->get_parameter("init_y").as_double();
        init_theta_ = this->get_parameter("init_theta").as_double();
    }

    void initSubscribers()
    {
        ekf_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/cmd_vel_imu", 10,
                std::bind(&BehaviorNode::ekfCallback, this, std::placeholders::_1));

        stm32_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/stm32_status", 10,
                std::bind(&BehaviorNode::stm32Callback, this, std::placeholders::_1));

        range_finder_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/range_finder", 10,
                std::bind(&BehaviorNode::rangeFinderCallback, this, std::placeholders::_1));

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
        recv_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "/button_gui",
            10,
            std::bind(&BehaviorNode::button_gui, this, std::placeholders::_1)
        );


        can_feedback_sub_ =this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/robot_state",
            10,
            std::bind(&BehaviorNode::can_feedback_callback, this, std::placeholders::_1
            )
        );

    }

    void initPublishers()
    {
        state_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/state_info", 10);

        pub_cmd_ = this->create_publisher<geometry_msgs::msg::Vector3>("/cmd_vel_custom", 10);

        logdata_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("/log_data", 10);
        
        // menual
        xyt_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("/xyt_gui",10);

        // Motor command
        motor_cmd = this->create_publisher<robot_msg::msg::Motor>("/motor_cmd", 10);
        //end menual
        ttt_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/ttt_cmd", 10);

    }

    void initTimers()
    {
        main_timer_ = this->create_wall_timer(std::chrono::milliseconds(5),
                std::bind(&BehaviorNode::behaviorLoop,this));
 
        ekf_timer_ = this->create_wall_timer(std::chrono::milliseconds(5),
                std::bind(&BehaviorNode::ekfLoop, this));

        control_timer_ = this->create_wall_timer(std::chrono::milliseconds(5),
                std::bind(&BehaviorNode::controlLoop,this));
        
        logdata_timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                std::bind(&BehaviorNode::logdata,this));
        
        // // //menual

        motor_pub = this->create_wall_timer(5ms, std::bind(&BehaviorNode::publish_motor_cmd, this));
        //vel_ramp = this->create_wall_timer(5ms, std::bind(&BehaviorNode::vel_ramp_fun, this));
        send_xyt_gui = this->create_wall_timer(5ms, std::bind(&BehaviorNode::xyt_gui_timer, this));
        // // //end menual
    }

    //--------------------------------trajectory-------------------------- 
    void loadTrajectoryByState()
    {
        std::string package_path = ament_index_cpp::get_package_share_directory("robot_pkg");
        std::string filename;
        
        if (stm32_rst_ == 1){
            switch (retry_state_)
            {
                case RETRY_TASK1:
                    filename = package_path + task_1;
                    traj_done = false;

                    break;

                case RETRY_TASK2:
                    filename = package_path + retry_task_2;
                    traj_done = false;

                    break;

                default:
                    break;
            }
        }

        if (auto_state_ == 0 && retry_state_ == RETRY_IDLE)
        {
            traj_data[0] = offset_x;
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
            filename = package_path + task_1;
            traj_done = false;
        }
        else if (auto_state_ == 2)
        {
            filename = package_path + task_2;
            traj_done = false;
        }

        if (filename.empty()) return;
        if (filename == current_trajectory_) return;

        current_trajectory_ = filename;
        trajectory_ = TrajectoryLoader::load(filename);
        traj_index_ = 0;

        RCLCPP_WARN(this->get_logger(),"Loaded trajectory: %s | points: %ld",
            filename.c_str(), trajectory_.size());
    }

    void runTrajectory()
    {
        if (manual_mode_) return;
        if (trajectory_.empty()) return;
        if (traj_done)return;
        auto &p = trajectory_[traj_index_];

        traj_data[0] = p.x;
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

    //-----------------------behavior loop-----------------------
    void behaviorLoop()
    {
        if (start_robot == false) return;
        choose_mode();
        size_play_ = (stm32_s_[2] == 0)? blue: red;
        size_play_problem();
        if (robot_mode_ == auto_mode){
            pose_control_enabled_ = true;
            heading_control_enabled_ = false;
            robot_auto();
        }
        else if (robot_mode_ == remote_mode){
            pose_control_enabled_ = false;
            heading_control_enabled_ = true;
            menual(); 
        }
    }

    //------------------------EKF--------------------
    void initEKF()
    {
        ekf_ = EKF();
        ekf_.setState(init_x_, init_y_, init_theta_);
    }
    void resetEKF()
    {
        ekf_ = EKF();
        ekf_.setState(
            init_x_,
            init_y_,
            init_theta_);

        x_enc = 0.0f;
        y_enc = 0.0f;
        prev_x_enc = 0.0f;
        prev_y_enc = 0.0f;
        have_enc_pose_ = false;
        have_prev_enc_pose_ = false;
        theta_imu = init_theta_;

        last_time_ = this->now();
        first_run_ = true;
    }

    void ekfLoop()
    {
        if (stm32_rst_ != last_reset_signal_)
        {
            size_play_ = (stm32_s_[2] == 0)? blue: red;
            init_x_ = offset_x;
            resetEKF();
            last_reset_signal_ = stm32_rst_;
        }

        if (first_run_)
        {
            last_time_ = this->now();
            first_run_ = false;
            return;
        }
        rclcpp::Time now = this->now();
        double dt =(now - last_time_).seconds();
        last_time_ = now;

        if (dt < 0.001) dt = 0.001;
        if (dt > 0.05)dt = 0.05;

        float vx_enc_calc = 0.0f;
        float vy_enc_calc = 0.0f;
        if (have_prev_enc_pose_) {
            double enc_dt = (enc_pose_time_ - prev_enc_pose_time_).seconds();
            if (enc_dt >= 1e-4) {
                vx_enc_calc = (x_enc - prev_x_enc) / static_cast<float>(enc_dt);
                vy_enc_calc = (y_enc - prev_y_enc) / static_cast<float>(enc_dt);
            }
        }

        ekf_.predict(vx_enc_calc, vy_enc_calc, theta_imu, static_cast<float>(dt));
        ekf_.update(theta_imu);
        auto state = ekf_.getState();
                //=========range_fuse
        auto laser_world = LaserToWorld::transform(range_finder[0], range_finder[1], range_finder[2], theta_imu, size_play_);
        y_new_ = WALL_Y - ROBOT_FRAME_Y - laser_world.y;
        x_new_ = ROBOT_FRAME_X + laser_world.x;

        float errory = fabsf(traj_data[1] - Y_);
        float errorx = fabsf(traj_data[0] - X_);
        correcting_ekf ();
        if (correct_ekf == true && errory < 0.1f) Y_ = y_new_;
        else Y_ = state[1];
        if (correct_ekf  == true && errorx < 0.1f) X_ = x_new_;    
        else X_= state[0];   
        if (correct_ekf == true && errory < 0.01f) x_est[1] = y_new_;
        if (correct_ekf  == true && errorx < 0.01f) x_est[0] = x_new_;

        // X_ = state[0];
        // Y_ = state[1];

        if (++log_counter_ >= 200)
        {
            // RCLCPP_INFO(this->get_logger(), "EKF -> x: %.3f y: %.3f th: %.3f, dt: %.3f", X_, Y_, theta_imu, dt );
            RCLCPP_INFO (this->get_logger(), "EKF -> x: %.3f y: %.3f th: %.3f", X_, Y_, theta_imu  );
            RCLCPP_INFO (this->get_logger(), "range -> y1: %.3f, y2: %.3f x: %.3f", range_finder[0], range_finder[1], range_finder[2]);
            RCLCPP_INFO(this->get_logger(), "Laser -> x: %.3f y: %.3f", x_new_, y_new_);
            // RCLCPP_INFO(this->get_logger(), "Laser -> x: %.3f y: %.3f", laser_world.x, laser_world.y);
            log_counter_ = 0;
        }
    }

    void controlLoop()
    {
        rclcpp::Time now = this->now();

        float dt =(now - last_time_).seconds();
        last_time_ = now;
        if (dt < 0.001)dt = 0.005;
        if (dt > 0.05) dt = 0.05;
        //---------- auto -> control pose
        if (pose_control_enabled_ == true)
        {
            heading_control_enabled_ = false;
            controller_.setReference(traj_data[0], traj_data[1], traj_data[2] );
            controller_.computeControl(X_, Y_, theta_imu, dt, traj_data[3], traj_data[4], traj_data[5], vx_, vy_, omega_);
        }
        // ---------- remote -> control heading only
        if (heading_control_enabled_ == true)
        {
            traj_data[0] = X_;
            traj_data[1] = Y_;
            pose_control_enabled_ = false;
            if (omega!=0){
                theta_ref = theta_imu;
                omega_ = omega;
            }
            else {
                omega_ = controller_.headingControl(theta_ref, theta_imu, dt);
            }
            vx_= cosf(theta_imu)*vx + sinf(theta_imu)*vy;
            vy_= -sinf(theta_imu)*vx+ cosf(theta_imu)*vy;
        }
        geometry_msgs::msg::Vector3 cmd;
        cmd.x = vx_ ;
        cmd.y = vy_ ;
        cmd.z = omega_;
        pub_cmd_->publish(cmd);
    }

    void xyt_gui_timer(void){
        float range_ygui = 0.0f;
        if (size_play_ == blue)
        {
            range_ygui = range_finder[0];
        }
        else range_ygui = range_finder[1];
        std_msgs::msg::Float32MultiArray msg;
        msg.data = {X_, Y_, theta_imu, range_finder[2], range_ygui};
        xyt_pub ->publish(msg);
    }
    //menual 

    void menual(void){
        
        test_gripper();
        change_speed();
        // other_comand();
        ttt_control();
 
    }

    void msg_callback(const robot_msg::msg::Gamepad::SharedPtr msg)
    {
        button = msg->btn;
        x5.setButton(msg->btn);
        x5.setMeihua(msg->btnm);
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
        ramp(vx, target_vx, ramp_step);
        ramp(vy, target_vy, ramp_step);
        ramp(omega, target_omega, ramp_step);

    }
    void choose_mode(void){
        if (x5.isMeihua(Menual)){
            robot_mode_ = remote_mode;
        }
        else if (x5.isMeihua(Auto)){
            robot_mode_ =auto_mode;
        }
    }
    void test_gripper(void) {
        // select number
        if      (x5.isPressed(X5_SHERE | X5_RB))    number = 0;
        else if (x5.isPressed(X5_SHERE | X5_RT))    number = 1;
        else if (x5.isPressed(X5_SHERE | X5_LB))    number = 2;
        else if (x5.isPressed(X5_SHERE | X5_MENU))    number = 3;
        else if (x5.isPressed(X5_SHERE | X5_B)){

        number = 4; state_Move_forward();}
        else if (x5.isPressed(X5_SHERE | X5_X)){

        number = 5; state_Move_backward();}
        else if (x5.isPressed(X5_SHERE | X5_Y)){

          number = 6; degree_up_to_5();}
        else if (x5.isPressed(X5_SHERE | X5_A)){
            number = 7; degree_down_to_95();}
        
        
        else if (x5.isMeihua(Home)){
            number =8;
        }
        else if (x5.isMeihua(chancel)){
            number =9;
        }
        
        // reset ticks on state change
        if (number != last_number) {
            delay_tick  = 0;
            delay_tick2 = 0;
            delay_tick3 = 0;
        }
        last_number = number;

        // run every tick — no state_entered_ guard
        if      (number == 0) state_IDLE();
        else if (number == 1) state_test();
        else if (number == 2) state_grip_staff();
        else if (number == 3) life_saving();
       RCLCPP_INFO(this->get_logger(),"numble=%d",number);
        
    }
    void change_speed(void){
        if (x5.isPressed(X5_LT)){
            ramp_step = 0.01;
            scaled = 0.2;
        RCLCPP_INFO(this->get_logger(),"robot_slow");
        }
        else{
            ramp_step = 0.01;
            scaled = 3.0;
           RCLCPP_INFO(this->get_logger(),"robot_fast_mode");
        }}

void ttt_control(void)
{
    // =================================================
    // RB FLOW
    // lift first
    // =================================================

    if(ttt_number == 1)
    {
        // STEP 0
        if(rb_step == 0)
        {
            send_can(
                fb_forward,
                45,
                35,
                0
            );

            if(fabs(fb_lift - 35) < 0.5f)
            {
                if(!rb_wait)
                {
                    step_time = this->now();
                    rb_wait = true;
                }

                if((this->now() - step_time).seconds() > 0.2)
                {
                    rb_step = 1;
                    rb_wait = false;
                }
            }
        }

        // STEP 1
        else if(rb_step == 1)
        {
            send_can(
                45,
                45,
                35,
                0
            );

            rb_step = 0;
            ttt_number = 0;
        }
    }

    // =================================================
    // RT FLOW
    // grip1 -> lift -> grip2 -> release grip1
    // =================================================

    else if(ttt_number == 2)
    {
        // STEP 0
        if(rt_step == 0)
        {
            send_can(
                fb_forward,
                17,
                fb_lift,
                0
            );

            if(fabs(fb_grip1 - 17) < 1.0f)
            {
                if(!rt_wait)
                {
                    step_time = this->now();
                    rt_wait = true;
                }

                if((this->now() - step_time).seconds() > 0.2)
                {
                    rt_step = 1;
                    rt_wait = false;
                }
            }
        }

        // STEP 1
        else if(rt_step == 1)
        {
            send_can(
                0,
                17,
                68,
                0
            );

            if(fabs(fb_lift - 68) < 1.0f)
            {
                if(!rt_wait)
                {
                    step_time = this->now();
                    rt_wait = true;
                }

                if((this->now() - step_time).seconds() > 0.3)
                {
                    rt_step = 2;
                    rt_wait = false;
                }
            }
        }

        // STEP 2
        else if(rt_step == 2)
        {
            send_can(
                0,
                17,
                68,
                35
            );

            if(fabs(fb_grip2 - 35) < 1.0f)
            {
                if(!rt_wait)
                {
                    step_time = this->now();
                    rt_wait = true;
                }

                if((this->now() - step_time).seconds() > 0.3)
                {
                    rt_step = 3;
                    rt_wait = false;
                }
            }
        }

        // STEP 3
        else if(rt_step == 3)
        {
            send_can(
                0,
                45,
                68,
                35
            );

            rt_step = 0;
            ttt_number = 0;
        }
    }

    // =================================================
    // HOME
    // =================================================

    if(ttt_number == 3)
    {
        send_can(
            0,
            45,
            0,
            0
        );

        ttt_number = 0;
    }

    // =================================================
    // LB FLOW
    // move forward safely
    // =================================================

    if(ttt_number == 4)
    {
        // STEP 0
        if(lb_step == 0)
        {
            send_can(
                fb_forward,
                45,
                10,
                35
            );

            if(!lb_wait)
            {
                step_time = this->now();
                lb_wait = true;
            }

            if((this->now() - step_time).seconds() > 0.5)
            {
                lb_step = 1;
                lb_wait = false;
            }
        }

        // STEP 1
        else if(lb_step == 1)
        {
            send_can(
                45,
                45,
                10,
                35
            );

            lb_step = 0;
            ttt_number = 0;
        }
    }

    // =================================================
    // B FLOW
    // grip1 toggle
    // =================================================

    if(ttt_number == 5)
    {
        if(!grip_closed)
        {
            send_can(
                fb_forward,
                17,
                fb_lift,
                fb_grip2
            );

            grip_closed = true;
        }
        else
        {
            send_can(
                fb_forward,
                30,
                fb_lift,
                fb_grip2
            );

            grip_closed = false;
        }

        ttt_number = 0;
    }

    // =================================================
    // LT FLOW
    // lower safely
    // =================================================

    if(ttt_number == 6)
    {
        // STEP 0
        if(lt_step == 0)
        {
            send_can(
                fb_forward,
                17,
                fb_lift,
                fb_grip2
            );

            if(fabs(fb_grip1 - 17) < 0.5f)
            {
                if(!lt_wait)
                {
                    step_time = this->now();
                    lt_wait = true;
                }

                if((this->now() - step_time).seconds() > 0.3)
                {
                    lt_step = 1;
                    lt_wait = false;
                }
            }
        }

        // STEP 1
        else if(lt_step == 1)
        {
            send_can(
                0,
                17,
                37,
                35
            );

            lt_step = 0;
            ttt_number = 0;
        }
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
        delay_tick0++;
        desire_deg = 0;
        desire_pose = 0;
        delay_tick=0;
        delay_tick2 =0;
        delay_tick3=0;
        sensor1 = 1;
        sensor2 = 0;
        sensor3 = 0;
        if (delay_tick0 >=200){
            sensor1 = 0;
        }
    }
    void state_grip_staff()
    {
        delay_tick++;

        if (delay_tick >= 100){
            sensor1 = 0;
        }
        if (delay_tick >= 200){
            sensor3 = 0;
            desire_deg = -90;  // only set once when tick reaches 200
        }
        // remove the "desire_deg = 0" from the top so other_comand can adjust it freely before tick 200

        if (degree_rev < -88){
            sensor2 = 1;
        } else {
            sensor2 = 0;
        }
    }
    void state_test(){
        sensor1 = 1;
        desire_pose = -0.049;

        if (pose_rev < -0.047)
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
            desire_pose -= 0.0005;

            if (desire_pose < -0.20)
            {
                desire_pose = -0.20;
            }
        }
    void state_Move_backward()
        {
            desire_pose += 0.0005;

            if (desire_pose > 0.0)
            {
                desire_pose = 0.0;
            }
        }
    void degree_up_to_5(void){
        desire_deg +=0.05;
        if (desire_deg>=10){
            desire_deg =10;
        }
    }
    void degree_down_to_95(void){
        desire_deg -=0.05;
        if (desire_deg<=-100){
            desire_deg =-100;
        }
    }


void send_can(float forward,
              float grip1,
              float lift,
              float grip2)
{
    std_msgs::msg::Float32MultiArray msg;

    msg.data = {
        forward, 
        grip1, 
        lift, 
        grip2
    };

    ttt_pub_->publish(msg);

    RCLCPP_INFO(this->get_logger(),
                "F: %.1f G1: %.1f L: %.1f G2: %.1f",
                forward,
                grip1,
                lift,
                grip2);
}



    //------------------------function that stop changing anymore------------------------
    void correcting_ekf(){
        if (traj_done == true && auto_state_ == 1){
            correct_ekf = true;
        }
        else {
            correct_ekf = false;
        }
    }
    // 
    void size_play_problem(void){
        std::string package_path = ament_index_cpp::get_package_share_directory("robot_pkg");
        std::string task_file[6] = {"/trajectory_file/task1_blue.txt", "/trajectory_file/task2_blue.txt", 
                                    "/trajectory_file/task1_red.txt", "/trajectory_file/task2_red.txt",
                                    "/trajectory_file/retry_task2_blue.txt", "/trajectory_file/retry_task2_red.txt"};
        
        switch (size_play_){
            case blue:
                offset_x = offsetx_blue;
                task_1 = task_file[0];
                task_2 = task_file[1];
                retry_task_2 = task_file[4];
                break;
            case red:
                offset_x = offsetx_red;
                task_1 = task_file[2];
                task_2 = task_file[3];
                retry_task_2 = task_file[5];
                break;
             default:
                break;
        }
    }   

    // sign for x reference based on the side playing
        float sign_xref (float x_ref)
    {
        if (size_play_ == blue)
            return x_ref;
        else
            return -x_ref;
    }
    // log data 
    void logdata(void){
        float ex = traj_data[0] - X_;
        float ey = traj_data[1] - Y_;
        float etheta = traj_data[2] - theta_imu;
        std_msgs::msg::Float32MultiArray msg;
        msg.data = {X_, Y_, theta_imu, 
                    traj_data[0], traj_data[1], traj_data[2], 
                    ex, ey, etheta};
        logdata_pub->publish(msg);  
    }
    // end 
    void robot_auto(){
    handleReset();
    runTrajectory();
    handleButton();
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

    // bunheang call back
        void ekfCallback(
        const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 3)
            return;

        if (have_enc_pose_) {
            prev_x_enc = x_enc;
            prev_y_enc = y_enc;
            prev_enc_pose_time_ = enc_pose_time_;
            have_prev_enc_pose_ = true;
        }

        x_enc = msg->data[0];
        y_enc = msg->data[1];
        enc_pose_time_ = this->now();
        have_enc_pose_ = true;

        theta_imu = wrapAngle(msg->data[2] + offset_theta);
        // laser_y_ = msg->data[3];
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

    void rangeFinderCallback(
        const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 3)
            return;

        range_finder[0] = msg->data[0];
        range_finder[1] = msg->data[1];
        range_finder[2] = msg->data[2];
    }

    void manualRefCallback(
        const geometry_msgs::msg::Pose2D::SharedPtr msg)
    {
        manual_mode_ = true;

        float x_ref = msg->x;
        pose_control_enabled_ = true;
        traj_data[0] = x_ref;
        traj_data[1] = msg->y;
        traj_data[2] = msg->theta;
        // traj_data[2] = yaw_deg * M_PI / 180.0f;

        traj_data[3] = 0.0f;
        traj_data[4] = 0.0f;
        traj_data[5] = 0.0f;
    }

};
void BehaviorNode::button_gui(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
{
    if (msg->data.empty()) return;
    uint8_t btn =
        (msg->data[0] << 0) |
        (msg->data[1] << 1) |
        (msg->data[2] << 2) |
        (msg->data[3] << 3) |
        (msg->data[4] << 4) |
        (msg->data[5] << 5) |
        (msg->data[6] << 6);

    gui_.setButton(btn);

    if (gui_.isPressed(ButtonguiLib::Start))
    {
        RCLCPP_INFO(this->get_logger(), "START pressed");
    }

    if (gui_.isPressed(ButtonguiLib::Manual))
    {
        RCLCPP_INFO(this->get_logger(), "MANUAL mode");
    }
    if (gui_.isPressed(ButtonguiLib::Debug))
    {
        RCLCPP_INFO(this->get_logger(), "Debug mode");
    }
    if (gui_.isPressed(ButtonguiLib::Auto))
    {
        RCLCPP_INFO(this->get_logger(), "Auto mode");
    }
        if (gui_.isPressed(ButtonguiLib::Switch_side))
    {
        RCLCPP_INFO(this->get_logger(), "Switch_side mode");
    }
    if (gui_.isPressed(ButtonguiLib::retry_zone1))
    {
        RCLCPP_INFO(this->get_logger(), "retry_zone1 mode");
    }
    if (gui_.isPressed(ButtonguiLib::retry_zone3))
    {
        RCLCPP_INFO(this->get_logger(), "retry_zone3 mode");
    }

}

void BehaviorNode::can_feedback_callback(
const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if(msg->data.size() < 4)
        return;

    fb_forward = msg->data[0];
    fb_grip1   = msg->data[1];
    fb_lift    = msg->data[2];
    fb_grip2   = msg->data[3];
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<BehaviorNode>());

    rclcpp::shutdown();

    return 0;
}