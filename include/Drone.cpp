#include "Drone.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

/**
 * @brief Send a command to Arm the vehicle
 */
void Drone::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void Drone::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

void Drone::goTo(float x, float y, float z) {
	publish_offboard_control_mode(true, false);
	publish_trajectory_setpoint(x,y,z);
}

void Drone::setVelocity(float vx, float vy, float vz) {
	publish_offboard_control_mode(false, true);
	TrajectorySetpoint msg{};
	msg.velocity = {vx,vy,vz};
	msg.yaw = -3.14; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void Drone::publish_offboard_control_mode(bool position, bool velocity)
{
	OffboardControlMode msg{};
	msg.position = position;
	msg.velocity = velocity;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void Drone::publish_trajectory_setpoint(float x, float y, float z)
{
	TrajectorySetpoint msg{};
	msg.position = {x,y,z};
	msg.yaw = -3.14; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void Drone::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

void Drone::positionCallback(const VehicleLocalPosition::SharedPtr msg) {
	positionReference = *msg;
}

Eigen::Vector3d Drone::getCurrentPosition() {
	return Eigen::Vector3d({
		positionReference.x,
		positionReference.y,
		positionReference.z
	});
}