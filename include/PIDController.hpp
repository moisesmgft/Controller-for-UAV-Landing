#include <chrono>

using namespace std::chrono;
class PIDController {
public:
    PIDController(float Kp, float Ki, float Kd) :
        _Kp(Kp), _Ki(Ki), _Kd(Kd), ITerm(0.0), _minWindup(-1e3f), _maxWindup(1e3f),  lastTime{system_clock::now()} 
    {}

    float getOutput(float current, float reference);
    void reset();

    /*
    Setter
    */
    void setWindup(float min, float max);

    /*
    Getters
    */
    float getKp() {return _Kp;}
    float getKd() {return _Kd;}
    float getKi() {return _Ki;}



private:

    float _Kp, _Ki, _Kd, ITerm, _minWindup, _maxWindup, lastError;
    system_clock::time_point lastTime;
};  