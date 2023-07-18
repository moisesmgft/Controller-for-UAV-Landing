#include <utility>
#include <random>
#include <functional>

# define PI 3.14159265358979323846

class Trajectory {
public:
    Trajectory(float a_, float v_, std::pair<float,float> center_, float mean, float stddev) : 
        a(a_), v(v_), c(center_),
        randomGenerator(std::bind(
                            std::normal_distribution<float>{mean, stddev},
                            std::mt19937(std::random_device{}())
                        ))
    {
        float totalTime = (6.09722*a) / v;
        p = 2 * PI / (totalTime);
    }
    std::pair<float,float> getPoint(float time);
    float getParam(float time);
private:
    float a, v, p;
    std::pair<float,float> c;
    std::function<float()> randomGenerator;

};
