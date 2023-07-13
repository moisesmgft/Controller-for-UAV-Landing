#include "PIDController.hpp"

Eigen::Matrix<float,4,5> PIDController::createMtxB() { 

    return (Eigen::Matrix<float,4,5>() <<
           0.0,             0.0,   2*(_Ts+2*_tau),  -4*_tau,  -2*(_Ts-2*_tau),
           0.0,             0.0, _Ts*(_Ts+2*_tau),  _Ts*_Ts, _Ts*(_Ts-2*_tau),
           0.0,             0.0,              4.0,     -4.0,              4.0,
        8*_tau,  2*(_Ts-2*_tau),              0.0,      0.0,              0.0).finished();
        
}

Eigen::Matrix<float,1,5> PIDController::createCoeffs() {
    Eigen::Matrix<float,1,4> A({_Kp, _Ki, _Kd, 1.0});
    return A*createMtxB();
}

float PIDController::getOutput(const Eigen::Vector3d current, float dt)
{
    return 0.0f;
}
