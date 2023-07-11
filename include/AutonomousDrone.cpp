#include "AutonomousDrone.hpp"

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <cmath>



#include <chrono>
#include <iostream>


using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

void AutonomousDrone::circlePath(Drone* drone,AutonomousDrone* solution) {

            
    float x = solution->radius * sin(solution->omega * drone->getTime());
    float y = solution->radius * cos(solution->omega * drone->getTime());

    drone->goTo(x,y,-5.0);

}

void AutonomousDrone::eight(Drone * drone, AutonomousDrone * solution) {
    auto [x,y] = solution->trajectory.getPoint(drone->getTime());
    drone->goTo(x,y,-5.0);
}

