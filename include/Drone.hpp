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


class AutonomousDrone; // resolve dependencia circular
class Drone : public rclcpp::Node
{
public:
	typedef void (*Callback)(Drone*, AutonomousDrone*);
	Drone(Callback _callback, AutonomousDrone* _solution) : Node("drone"), callback(_callback), solutionPtr(_solution)
	{

		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);


		offboard_setpoint_counter_ = 0;

		time_count_ = 0;
		/*
		auto timer_callback = [this]() -> void {

			if (offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				// Arm the vehicle
				this->arm();
				
			}

			this->goTo(0.0,0.0,-5.0);

			// stop the counter after reaching 11
			offboard_setpoint_counter_++;
			
		};
		*/

		auto func = [this]() -> void {
			this->time_count_ += 0.01;
			callback(this, this->solutionPtr);
		};
		int i = 10;
		while(--i)
			this->goTo(0.0,0.0,-5.0);
		this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
		this->arm();
		timer_ = this->create_wall_timer(10ms, func);
	}

	void arm();
	void disarm();
	void goTo(float x, float y, float z);


	// Getters
	float getTime() {return time_count_;}


private:

	Callback callback;
	AutonomousDrone* solutionPtr;


	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

	//std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent
	float time_count_;

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint(float x, float y, float z);
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

	

};


#endif