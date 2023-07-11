#include <utility>

# define PI 3.14159265358979323846

class Trajectory {
public:
    Trajectory(float a_, float v_, std::pair<float,float> center_) : a(a_), v(v_), c(center_) {
        float totalTime = (6.09722*a) / v;
        p = 2 * PI / (totalTime);
    }
    std::pair<float,float> getPoint(float time);
    float getParam(float time);
private:
    float a;
    float v;
    std::pair<float,float> c;
    float p;
};
