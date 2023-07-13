#include <Eigen/Dense>

class PIDController {
public:
    PIDController(float Kp, float Ki, float Kd, float tau, float Ts) :
        _Kp(Kp), _Ki(Ki), _Kd(Kd), _tau(tau), _Ts(Ts), _coeffs{_coeffs} {}

    /*
    Getters
    */
    float getKp() {return _Kp;}
    float getKd() {return _Kd;}
    float getKi() {return _Ki;}
    float getTau() {return _tau;}

private:

    float _Kp, _Ki, _Kd, _tau, _Ts;
    Eigen::Matrix<float,1,5> _coeffs;

    Eigen::Matrix<float,4,5> createMtxB();
    Eigen::Matrix<float,1,5> createCoeffs();
};  