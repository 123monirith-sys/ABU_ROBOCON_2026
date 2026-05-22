    #include "robot_pkg/ekf_lib.hpp"
    #include <cmath>
       float x_est[3] = {0.0f, 0.0f, 0.0f};
    EKF::EKF()
    {
 
        // for (int i = 0; i < 3; i++)
        //     x_est[i] = x_est_init[i];
    

        float P_init[3][3] = {
            {0.01f, 0.0f, 0.0f},
            {0.0f, 0.01f, 0.0f},
            {0.0f, 0.0f, 0.01f}
        };

        float Q_init[3][3] = {
            {0.001f, 0.0f, 0.0f},
            {0.0f, 0.001f, 0.0f},
            {0.0f, 0.0f, 0.005f}
        };

        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
            {
                P[i][j] = P_init[i][j];
                Q[i][j] = Q_init[i][j];
            }

        R = 0.01f;
    }

    void EKF::predict(float vx, float vy, float theta, float dt)
    {
        (void)theta;
        float th = -theta;

        x_est[0] += (vx * std::cos(th) + vy * std::sin(th)) * dt;
        x_est[1] += (-vx * std::sin(th) + vy * std::cos(th)) * dt;

        float F[3][3] = {
            {1, 0, (-vx * std::sin(th) - vy * std::cos(th)) * dt},
            {0, 1, ( vx * std::cos(th) - vy * std::sin(th)) * dt},
            {0, 0, 1}
        };

        float P_temp[3][3] = {0};
        float P_new[3][3] = {0};

        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                for (int k = 0; k < 3; k++)
                    P_temp[i][j] += F[i][k] * P[k][j];

        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                for (int k = 0; k < 3; k++)
                    P_new[i][j] += P_temp[i][k] * F[j][k];

        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                P[i][j] = P_new[i][j] + Q[i][j];
    }

    void EKF::update(float theta_meas)
    {
        float y = theta_meas - x_est[2];

        while (y > M_PI)
            y -= 2.0f * M_PI;
        while (y < -M_PI)
            y += 2.0f * M_PI;

        float S = P[2][2] + R;

        float K[3];
        for (int i = 0; i < 3; i++)
            K[i] = P[i][2] / S;

        for (int i = 0; i < 3; i++)
            x_est[i] += K[i] * y;

        while (x_est[2] > M_PI)
            x_est[2] -= 2.0f * M_PI;
        while (x_est[2] < -M_PI)
            x_est[2] += 2.0f * M_PI;

        float P_new[3][3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                P_new[i][j] = P[i][j] - K[i] * P[2][j];

        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                P[i][j] = P_new[i][j];
    }

    void EKF::setState(float x, float y, float theta)
    {
        x_est[0] = x;
        x_est[1] = y;
        x_est[2] = theta;

        P[0][0] = 0.01f; P[0][1] = 0.0f; P[0][2] = 0.0f;
        P[1][0] = 0.0f;  P[1][1] = 0.01f; P[1][2] = 0.0f;
        P[2][0] = 0.0f;  P[2][1] = 0.0f;  P[2][2] = 0.01f;
    }

    std::array<float, 3> EKF::getState() const
    {
        return {x_est[0], x_est[1], x_est[2]};
    }