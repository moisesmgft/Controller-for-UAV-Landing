#include "Drone.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Drone>());
	rclcpp::shutdown();
	return 0;
}

/*
int main() {
	return 0;
}
*/