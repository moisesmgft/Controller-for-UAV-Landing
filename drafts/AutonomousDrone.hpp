#ifndef AUTONOMOUS_DRONE_H
#define AUTONOMOUS_DRONE_H

#include "Drone.hpp"
#include "Trajectory.hpp"
#include "MultiAxisPIDController.hpp"

class AutonomousDrone
{
private:
    Trajectory trajectory;
    MultiAxisPIDController controller;

public:
    AutonomousDrone(Eigen::Vector3d horizontalGains, Eigen::Vector3d verticalGains) : 
        trajectory(10.0f,9.4f,{8.0f,8.0f}),
        controller(horizontalGains, verticalGains)
    {}

    static void hover(Drone*, AutonomousDrone*);
    static void circlePath(Drone*,AutonomousDrone*);
    static void eight(Drone*, AutonomousDrone*);
    //static void horizontalStepResponse(Drone*, AutonomousDrone*);
    float radius = 5.0;
    float omega = 0.5;
};

#endif