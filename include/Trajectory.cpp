#include "Trajectory.hpp"
#include <cmath>

using namespace std::chrono;

/**
 * @brief Get the point that describes the trajectory in the current time.
 * xNoise and yNoise is the point with noise.
 * xTrue and yTrue is the point where the base actually is.
 */
std::tuple<float,float,float,float> Trajectory::getPoint()
{
    float param = getParam();
    float xTrue = c.first/2.0f + a * sin(param),
          yTrue = c.second/2.0f + a * sin(param) * cos(param);
    float xNoise = xTrue + randomGenerator(),
          yNoise = yTrue + randomGenerator();
    return {xNoise, yNoise, xTrue, yTrue};
}

/**
 * @brief Utillity function to calculate the next point.
 */
float Trajectory::getParam()
{
    system_clock::time_point currentTime = system_clock::now();
    duration<float> duration = currentTime - startTime;
    float dt = duration.count();
    return p * dt;
}
