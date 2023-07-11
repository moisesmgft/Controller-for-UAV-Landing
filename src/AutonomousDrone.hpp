#include "Drone.hpp"

class AutonomousDrone : Drone
{
private:
    void pathPlanning() override;
    uint64_t offboard_setpoint_counter_;
public:
    AutonomousDrone() : Drone() {
        offboard_setpoint_counter_ = 0;
    }
};
