#include "Trajectory.hpp"
#include <cmath>

using namespace std::chrono;

std::tuple<float,float,float,float> Trajectory::getPoint()
{
    float param = getParam();
    float xTruth = c.first/2.0f + a * sin(param),
          yTruth = c.second/2.0f + a * sin(param) * cos(param);
    float xNoise = xTruth + randomGenerator(),
          yNoise = yTruth + randomGenerator();
    return {xNoise, yNoise, xTruth, yTruth};
}

float Trajectory::getParam()
{
    system_clock::time_point currentTime = system_clock::now();
    duration<float> duration = currentTime - startTime;
    float dt = duration.count();
    return p * dt;
}
