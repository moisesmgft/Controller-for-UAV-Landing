#include "MultiAxisPIDController.hpp"

Eigen::Vector3d MultiAxisPIDController::getOutput(const Eigen::Vector3d &current) {
    return Eigen::Vector3d({
        _xController.getOutput(current[0]),
        _yController.getOutput(current[1]),
        _zController.getOutput(current[2])
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

void MultiAxisPIDController::setReference(const Eigen::Vector3d &reference) {
    _xController.setReference(reference[0]);
    _yController.setReference(reference[1]);
    _zController.setReference(reference[2]);
}

