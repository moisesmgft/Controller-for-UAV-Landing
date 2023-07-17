#ifndef DRONE_H
#define DRONE_H


#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <cmath>

#include <chrono>
#include <iostream>
#include <Eigen/Eigen>


using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using std::placeholders::_1;



class Drone : public rclcpp::Node
{
public:
	Drone() : Node("drone")
	{

		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);


		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		vehicle_local_position_subscription_ = this->create_subscription<VehicleLocalPosition>(
			"fmu/out/vehicle_local_position", qos, std::bind(&Drone::positionCallback,this,_1));


		offboard_setpoint_counter_ = 0;
		time_count_ = 0;


		int i = 15;
		while(--i)
			this->goTo(0.0,0.0,-5.0);
		this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

		this->arm();
		

		//timer_ = this->create_wall_timer(10ms, func);
	}

	void arm();
	void disarm();
	void goTo(float x, float y, float z);


	// Getters
	int getCount() {return offboard_setpoint_counter_;}
	float getTime() {return time_count_;}
	Eigen::Vector3d getCurrentPosition();

private:



	rclcpp::TimerBase::SharedPtr timer_;

	/*
	Publishers
	*/
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

	/*
	Subscriptions
	*/
	rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_local_position_subscription_;

	//std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent
	float time_count_;

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint(float x, float y, float z);
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	/*
	Reference
	*/
	VehicleLocalPosition positionReference;

	/*
	Callbacks
	*/
	void positionCallback(const VehicleLocalPosition::SharedPtr);


	

};


#endif