#include "robot_pkg/laserToworld.hpp"

// Helper function to normalize angle to [-π, π]
static float normalizeAngle(float angle)
{
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

// Helper function to apply angle-dependent correction
// Corrects measurements when not at cardinal angles (0°, 90°, 180°, 270°)
static float applyAngleCorrection(float measurement, float theta)
{
    // Normalize theta to [-π, π]
    float normalized_theta = normalizeAngle(theta);
    
    // Find angle deviation from nearest cardinal direction
    // Cardinal angles: -π, -π/2, 0, π/2
    float min_deviation = M_PI; // Maximum possible deviation
    
    // Check deviation from each cardinal angle
    float cardinal_angles[] = {-M_PI, -M_PI/2, 0, M_PI/2};
    for (int i = 0; i < 4; i++) {
        float deviation = fabsf(normalized_theta - cardinal_angles[i]);
        if (deviation < min_deviation) {
            min_deviation = deviation;
        }
    }
    
    // If deviation is small (within ~10°), it's close to a cardinal angle
    if (min_deviation < 0.1745f) { // ~10 degrees in radians
        return measurement;
    }
    
    // Apply correction: the perpendicular distance is measurement * cos(deviation)
    // So corrected measurement = measurement * cos(deviation)
    float correction_factor = std::cos(min_deviation);
    
    return measurement * correction_factor;
}

LaserToWorld::Vector2D
LaserToWorld::transform(float range_y1,
                        float range_y2,
                        float range_x,
                        float theta, uint8_t size_play)
{
    Vector2D v;
    float range_y = (size_play == 0) ? range_y1 : range_y2;
    
    // Apply angle-dependent correction to measurements
    range_x = applyAngleCorrection(range_x, theta);
    range_y = applyAngleCorrection(range_y, theta);
    
    float c = std::cos(theta);
    float s = std::sin(theta);

    v.x = fabsf(range_x * c - range_y * s);
    v.y = fabsf(range_x * s + range_y * c);

    return v;
}