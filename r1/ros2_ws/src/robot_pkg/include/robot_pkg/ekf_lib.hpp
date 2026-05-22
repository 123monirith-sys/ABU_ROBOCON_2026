#ifndef EKF_LIB_HPP
#define EKF_LIB_HPP

#include <array>
extern float x_est[3];   // x, y, theta
class EKF
{
public:
    EKF();

    void predict(float vx, float vy, float theta, float dt);
    void update(float theta_meas);
    void setState(float x, float y, float theta);

    std::array<float, 3> getState() const;

private:
    // float x_est[3];
    float P[3][3];
    float Q[3][3];
    float R;
};

#endif