#include "Trajectory.hpp"
#include <cmath>

using namespace std::chrono;

std::pair<float,float> Trajectory::getPoint()
{
    float param = getParam();
    float x = c.first/2.0f + a * sin(param) + randomGenerator();
    float y = c.second/2.0f + a * sin(param) * cos(param) + randomGenerator();
    return {x, y};
}

float Trajectory::getParam()
{
    system_clock::time_point currentTime = system_clock::now();
    duration<float> duration = currentTime - startTime;
    float dt = duration.count();
    return p * dt;
}
