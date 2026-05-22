#ifndef POSE_CONTROL_LIB_HPP
#define POSE_CONTROL_LIB_HPP

class PoseControl
{
public:
    PoseControl();

    // =========================================================
    // SET REFERENCE
    // =========================================================
    void setReference(float x,
                      float y,
                      float yaw_deg);

    // =========================================================
    // MAIN CONTROL
    // =========================================================
    void computeControl(float X,
                        float Y,
                        float theta,
                        float dt,
                        float vx_traj,
                        float vy_traj,
                        float w_traj,
                        float &vx_r,
                        float &vy_r,
                        float &omega);

    // =========================================================
    // HEADING ONLY CONTROL
    // =========================================================
    float headingControl(float theta_ref,
                         float theta_meas,
                         float dt);

private:

    // =========================================================
    // REFERENCE
    // =========================================================
    float x_ref_;
    float y_ref_;
    float yaw_ref_;

    // =========================================================
    // PID GAINS
    // =========================================================
    float Kpx_;
    float Kdx_;

    float Kpy_;
    float Kdy_;

    float Kpt_;
    float Kdt_;

    // =========================================================
    // LIMITS
    // =========================================================
    float max_v_;
    float max_w_;

    // =========================================================
    // PREVIOUS ERRORS
    // =========================================================
    float ex_prev_;
    float ey_prev_;
    float et_prev_;

    // =========================================================
    // FILTERED OUTPUTS
    // =========================================================
    float vx_f_;
    float vy_f_;
    float w_f_;

    // =========================================================
    // FILTER COEFFICIENTS
    // =========================================================
    float alpha_v_;
    float alpha_w_;

    // =========================================================
    // UTILITY
    // =========================================================
    float wrapAngle(float angle);
};

#endif