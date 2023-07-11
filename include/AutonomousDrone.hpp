#ifndef AUTONOMOUS_DRONE_H
#define AUTONOMOUS_DRONE_H

#include "Drone.hpp"
#include "Trajectory.hpp"

class AutonomousDrone
{
private:
    Trajectory trajectory;
public:
    AutonomousDrone() : trajectory{10.0f,9.4f,{8.0f,8.0f}}
    {}

    static void circlePath(Drone*,AutonomousDrone*);
    static void eight(Drone*, AutonomousDrone*);
    float radius = 5.0;
    float omega = 0.5;
};

#endif