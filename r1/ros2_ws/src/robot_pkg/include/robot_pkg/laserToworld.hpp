#ifndef LASERTOWORLD_HPP
#define LASERTOWORLD_HPP

#include <cmath>
#include <cstdint>

class LaserToWorld
{
public:
    struct Vector2D
    {
        float x;
        float y;
    };

    static Vector2D transform(float range_x1, float range_x2, float range_y, float theta, uint8_t size_play);
                          
};

#endif