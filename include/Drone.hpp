#ifndef DRONE_H
#define DRONE_H


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


class Solution; // resolve dependencia circular
class Drone : public rclcpp::Node
{
public:
	typedef void (*Callback)(Drone*, Solution*);
	Drone(float omega_, float radius_, float dt_, Callback _callback) : Node("drone"), omega(omega_), radius(radius_), dt(dt_), callback(_callback)
	{

		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);


		offboard_setpoint_counter_ = 0;

		auto timer_callback = [this]() -> void {

			if (offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				// Arm the vehicle
				this->arm();
				
			}

			// offboard_control_mode needs to be paired with trajectory_setpoint

			float x = this->radius * cos(offboard_setpoint_counter_ * (dt/1000));
			float y = this->radius * sin(offboard_setpoint_counter_ * (dt/1000));

			publish_offboard_control_mode();
			this->goTo(x,y,-5.0);

			// stop the counter after reaching 11
			offboard_setpoint_counter_++;
			
		};

		timer_ = this->create_wall_timer(10ms, timer_callback);
	}

	void arm();
	void disarm();
	void goTo(float x, float y, float z);


private:


	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

	//std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

	float omega, radius, dt;

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint(float x, float y, float z);
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

	
	Callback callback;

};


#endif