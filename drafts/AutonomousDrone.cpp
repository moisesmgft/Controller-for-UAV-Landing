#include "AutonomousDrone.hpp"

#include <cmath>

void AutonomousDrone::hover(Drone* drone, AutonomousDrone* solution) {
    drone->goTo(0,0,-2.0);
}


void AutonomousDrone::circlePath(Drone* drone,AutonomousDrone* solution) {

            
    float x = solution->radius * sin(solution->omega * drone->getTime());
    float y = solution->radius * cos(solution->omega * drone->getTime());

    drone->goTo(x,y,-5.0);

}

void AutonomousDrone::eight(Drone * drone, AutonomousDrone * solution) {
    auto [x,y] = solution->trajectory.getPoint(drone->getTime());
    drone->goTo(x,y,-5.0);
}

