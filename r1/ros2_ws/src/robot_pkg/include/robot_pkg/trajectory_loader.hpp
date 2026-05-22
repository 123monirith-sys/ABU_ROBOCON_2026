#ifndef TRAJECTORY_LOADER_HPP
#define TRAJECTORY_LOADER_HPP

#include <vector>
#include <string>

struct TrajectoryPoint
{
    double time;
    double x;
    double y;
    double theta;
    double vx;
    double vy;
    double omega;
};

class TrajectoryLoader
{
public:
    static std::vector<TrajectoryPoint>
    load(const std::string &filename);

    static std::string
    selectTrajectoryFile(int mode);
};

#endif