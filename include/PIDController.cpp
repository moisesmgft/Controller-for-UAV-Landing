#include "PIDController.hpp"

using namespace std::chrono;

/**
 * @brief Get the controller output
 */
float PIDController::getOutput(float current, float reference) {

    system_clock::time_point currentTime = system_clock::now();
    duration<float> duration = currentTime - lastTime;
    float dt = duration.count();

    float error = reference - current;

    // P Term
    float PTerm = _Kp*error;
    
    // I Term
    ITerm += _Ki * (error + lastError) * dt / 2.0;
    // D Term
    float DTerm = _Kd * (error - lastError) / dt;

    lastError = error;
    lastTime = currentTime;

    float output = (PTerm + ITerm + DTerm);
    output = (output > _maxWindup) ? (_maxWindup) : ((output < _minWindup) ? (_minWindup) : (output));


    return output;
}

/**
 * @brief Reset the controller.
 */
void PIDController::reset() {
    lastError = 0.0f;
    ITerm = 0.0f;
    lastTime = system_clock::now();
}

/**
 * @brief Set the windup
 */
void PIDController::setWindup(float min, float max) { 
    _minWindup = min;
    _maxWindup = max;
}


