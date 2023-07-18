#include "Drone.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <vector>
#include <tuple>

int main(int argc, char *argv[])
{
	
	int i=2;
	while (--i) {

		rclcpp::init(argc, argv);

		auto drone = std::make_shared<Drone>();


		while (rclcpp::ok()) {
			drone->goTo(0.0,0.0,-3.0);
			rclcpp:spin_some(drone);
		}
		//rclcpp::spin(drone);

		rclcpp::shutdown();
		
	}
	
	

	return 0;
}