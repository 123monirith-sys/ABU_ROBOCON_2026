#include "robot_pkg/trajectory_loader.hpp"

#include <fstream>
#include <sstream>
#include <iostream>

std::vector<TrajectoryPoint>
TrajectoryLoader::load(const std::string &filename)
{
    std::vector<TrajectoryPoint> trajectory;

    std::ifstream file(filename);

    if (!file.is_open())
    {
        std::cerr << "Cannot open file: "
                  << filename << std::endl;

        return trajectory;
    }

    std::string line;

    while (std::getline(file, line))
    {
        if (line.empty())
            continue;

        std::stringstream ss(line);

        TrajectoryPoint point;

        ss >> point.time
           >> point.x
           >> point.y
           >> point.theta
           >> point.vx
           >> point.vy
           >> point.omega;

        if (!ss.fail())
        {
            trajectory.push_back(point);
        }
    }

    file.close();

    return trajectory;
}

std::string
TrajectoryLoader::selectTrajectoryFile(int mode)
{
    switch (mode)
    {
    case 0:
        return "trajectory_file/retry_task2.txt";

    case 1:
        return "trajectory_file/task1_red.txt";

    case 2:
        return "trajectory_file/task2_red.txt";

    }
}