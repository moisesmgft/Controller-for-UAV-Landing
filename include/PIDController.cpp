#include "PIDController.hpp"

using namespace std::chrono;

float PIDController::getOutput(float current) {

    system_clock::time_point currentTime = system_clock::now();
    duration<float> duration = currentTime - lastTime;
    float dt = duration.count();

    float error = _reference - current;

    // P Term
    float PTerm = _Kp*error;
    
    // I Term
    ITerm += _Ki * (error + lastError) * dt / 2.0;

    // D Term
    float DTerm = _Kd * (error - lastError) / dt;

    lastError = error;
    lastTime = currentTime;



    return (PTerm + ITerm + DTerm);
}

void PIDController::reset() {
    ITerm = 0.0f;
    lastTime = system_clock::now();
}

void PIDController::setWindup(float windup) { _windup = windup; }
void PIDController::setReference(float reference) { 
    _reference = reference;
    reset();
}


