#include "PIDController.hpp"
#include <Eigen/Dense>

class MultiAxisPIDController {
public:
    MultiAxisPIDController(Eigen::Vector3d horizontalGains, Eigen::Vector3d verticalGains) :
        _xController(horizontalGains[0], horizontalGains[1], horizontalGains[2]),
        _yController(horizontalGains[0], horizontalGains[1], horizontalGains[2]),
        _zController(verticalGains[0], verticalGains[1], verticalGains[2]) {}

    Eigen::Vector3d getOutput(const Eigen::Vector3d &current, const Eigen::Vector3d &reference);
    void reset();

    /*
    Setter
    */
    void setWindup(float horizontalWindup, float verticalWindup);


private:

    PIDController _xController, _yController, _zController;
};  