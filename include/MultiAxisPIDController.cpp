#include "MultiAxisPIDController.hpp"

Eigen::Vector3d MultiAxisPIDController::getOutput(const Eigen::Vector3d &current, const Eigen::Vector3d &reference) {
    return Eigen::Vector3d({
        _xController.getOutput(current[0], reference[0]),
        _yController.getOutput(current[1], reference[1]),
        _zController.getOutput(current[2], reference[2])
    });
}

void MultiAxisPIDController::reset() {
    _xController.reset();
    _yController.reset();
    _zController.reset();
}

void MultiAxisPIDController::setWindup(float horizontalWindup, float verticalWindup) {
    _xController.setWindup(horizontalWindup);
    _yController.setWindup(horizontalWindup);
    _zController.setWindup(verticalWindup);
}

