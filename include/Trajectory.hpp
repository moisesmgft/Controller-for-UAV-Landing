#include <utility>
#include <random>
#include <functional>
#include <chrono>

using namespace std::chrono;

# define PI 3.14159265358979323846

class Trajectory {
public:
    Trajectory(float a_, float v_, std::pair<float,float> center_, float mean, float stddev) : 
        a(a_), v(v_), c(center_), startTime(system_clock::now()),
        randomGenerator(std::bind(
                            std::normal_distribution<float>{mean, stddev},
                            std::mt19937(std::random_device{}())
                        ))
    {
        float totalTime = (6.09722*a) / v;
        p = 2 * PI / (totalTime);
    }
    std::pair<float,float> getPoint();
    float getParam();
private:
    float a, v, p;
    std::pair<float,float> c;
    system_clock::time_point startTime;
    std::function<float()> randomGenerator;

};
