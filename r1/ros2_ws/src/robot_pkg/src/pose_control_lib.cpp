#include "robot_pkg/pose_control_lib.hpp"

#include <algorithm>
#include <cmath>

#define DEG2RAD 0.0174532925f

PoseControl::PoseControl()
{
    // ===== Reference =====
    x_ref_ = 0.0f;
    y_ref_ = 0.0f;
    yaw_ref_ = 0.0f;

    // ===== Gains =====
    Kpx_ = 0.5f;
    Kdx_ = 0.0f;

    Kpy_ = 0.5f;
    Kdy_ = 0.0f;

    Kpt_ = 1.75f;
    Kdt_ = 0.03f;

    // ===== Limits =====
    max_v_ = 1.0f;
    max_w_ = 3.14f;

    // ===== Previous Errors =====
    ex_prev_ = 0.0f;
    ey_prev_ = 0.0f;
    et_prev_ = 0.0f;

    // ===== Velocity Filters =====
    vx_f_ = 0.0f;
    vy_f_ = 0.0f;
    w_f_  = 0.0f;

    // ===== Filter Coefficients =====
    alpha_v_ = 0.2f;
    alpha_w_ = 0.2f;
}

void PoseControl::setReference(float x, float y, float yaw_rad)
{
    x_ref_ = x;
    y_ref_ = y;
    yaw_ref_ = yaw_rad;
}

float PoseControl::wrapAngle(float angle)
{
    while (angle > M_PI)
        angle -= 2.0f * M_PI;

    while (angle < -M_PI)
        angle += 2.0f * M_PI;

    return angle;
}

void PoseControl::computeControl(float X,
                                 float Y,
                                 float theta,
                                 float dt,
                                 float vx_traj,
                                 float vy_traj,
                                 float w_traj,
                                 float &vx_r,
                                 float &vy_r,
                                 float &omega)
{
    // ===== Protect dt =====
    if (dt < 1e-4f)
        dt = 1e-4f;

    // POSITION ERROR (WORLD FRAME)
    float ex = x_ref_ - X;
    float ey = y_ref_ - Y;

    // DERIVATIVE ERROR
    float dex = (ex - ex_prev_) / dt;
    float dey = (ey - ey_prev_) / dt;

    // HEADING ERROR
    float etheta = wrapAngle(yaw_ref_  - theta);
    float det    = (etheta - et_prev_) / dt;

    // PD FEEDBACK
    float vx_pid = Kpx_ * ex + Kdx_ * dex;
    float vy_pid = Kpy_ * ey + Kdy_ * dey;
    float w_pid  = Kpt_ * etheta + Kdt_ * det;

    // FEEDFORWARD + FEEDBACK
    float vx_w = vx_traj + vx_pid;
    float vy_w = vy_traj + vy_pid;

    omega = w_traj + w_pid;

    // WORLD FRAME -> ROBOT FRAME
    vx_r = cosf(theta) * vx_w + sinf(theta) * vy_w;

    vy_r = -sinf(theta) * vx_w + cosf(theta) * vy_w;

    // SMART STOP
    if (fabs(ex) < 0.01f && fabs(ey) < 0.01f)
    {
        vx_r = 0.0f;
        vy_r = 0.0f;
    }
    // if (fabsf(vy))
    if (fabs(etheta) < (1.0f * DEG2RAD))
    {
        omega = 0.0f;
    }

    // LOW PASS FILTER
    vx_f_ = alpha_v_ * vx_r + (1.0f - alpha_v_) * vx_f_;

    vy_f_ = alpha_v_ * vy_r + (1.0f - alpha_v_) * vy_f_;

    // w_f_ = alpha_w_ * omega + (1.0f - alpha_w_) * w_f_;

    // SATURATION
    vx_f_ = std::max(-max_v_, std::min(max_v_, vx_f_));

    vy_f_ = std::max(-max_v_, std::min(max_v_, vy_f_));
    omega = std::max(-max_w_, std::min(max_w_, omega));

    // w_f_ = std::max(-max_w_, std::min(max_w_, w_f_));

    // OUTPUT
    vx_r = vx_f_;
    vy_r = vy_f_;
    omega = omega;

    // SAVE PREVIOUS
    ex_prev_ = ex;
    ey_prev_ = ey;
    et_prev_ = etheta;
}
// HEADING ONLY CONTROL
float PoseControl::headingControl(float theta_ref,
                                  float theta_meas,
                                  float dt)
{
    if (dt < 1e-4f)
        dt = 1e-4f;

    float Kpt = 3.0f;
    float Kdt = 0.3f;

    static float ew_prev = 0.0f;

    float ew = wrapAngle(theta_ref - theta_meas);

    float dew = (ew - ew_prev) / dt;

    float w_out = Kpt * ew + Kdt * dew;

    // ===== Stop Condition =====
    if (fabs(ew) < (1.0f * DEG2RAD))
    {
        w_out = 0.0f;
    }

    // ===== Filter =====
    static float w_filter = 0.0f;

    w_filter = alpha_w_ * w_out +
               (1.0f - alpha_w_) * w_filter;

    // ===== Saturation =====
    w_filter = std::max(-max_w_,
                        std::min(max_w_, w_filter));

    ew_prev = ew;

    return w_filter;
}