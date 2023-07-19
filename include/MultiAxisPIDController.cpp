#include "MultiAxisPIDController.hpp"

/**
 * @brief Get the output for each axis
 */
Eigen::Vector3d MultiAxisPIDController::getOutput(const Eigen::Vector3d &current, const Eigen::Vector3d &reference) {
    return Eigen::Vector3d({
        _xController.getOutput(current[0], reference[0]),
        _yController.getOutput(current[1], reference[1]),
        _zController.getOutput(current[2], reference[2])
    });
}

/**
 * @brief Reset each controller
 */
void MultiAxisPIDController::reset() {
    _xController.reset();
    _yController.reset();
    _zController.reset();
}

/**
 * @brief Set windup for each controller
 */
void MultiAxisPIDController::setWindup(Eigen::Vector2d horizontalWindup, Eigen::Vector2d verticalWindup) {
    _xController.setWindup(horizontalWindup[0], horizontalWindup[1]);
    _yController.setWindup(horizontalWindup[0], horizontalWindup[1]);
    _zController.setWindup(verticalWindup[0], verticalWindup[1]);
}

