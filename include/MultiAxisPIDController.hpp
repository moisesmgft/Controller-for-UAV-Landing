#include "PIDController.hpp"
#include <Eigen/Dense>

class MultiAxisPIDController {
public:
    MultiAxisPIDController(Eigen::Vector4d horizontalGains, Eigen::Vector4d verticalGains, float Ts) :
        _hController(horizontalGains[0], horizontalGains[1], horizontalGains[2], horizontalGains[3], Ts),
        _vController(verticalGains[0], verticalGains[1], verticalGains[2], verticalGains[3], Ts) {}

    


private:

    PIDController _hController, _vController;
};  