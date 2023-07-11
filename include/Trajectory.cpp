#include "Trajectory.hpp"
#include <cmath>

std::pair<float,float> Trajectory::getPoint(float time)
{
    float param = getParam(time);
    return {c.first/2.0f + a * sin(param), c.second/2.0f + a * sin(param) * cos(param)};
}

float Trajectory::getParam(float time)
{
    return p * time;
}
