#include "Drone.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <vector>
#include <tuple>

#include "Trajectory.hpp"
#include "FSM.hpp"
#include "MultiAxisPIDController.hpp"

#include <chrono>
using namespace std::chrono;




class stateTakeOff : public State
{
private:
	Drone* drone_;
public:
	void act() override {
	drone_->goTo(0.0,0.0,-5.0);
	//drone_->arm();
}
	bool to_stateTracking() {
	Eigen::Vector3d diff = Eigen::Vector3d({0.0,0.0,-5.0}) - drone_->getCurrentPosition();
	return diff.norm() < 0.05;
}
	stateTakeOff(Drone* drone) : drone_(drone) {}
};

class stateTracking : public State
{
private:
	Drone* drone_;
	MultiAxisPIDController controller_;
	Trajectory trajectory_;
	system_clock::time_point startTime;
	bool enter;
public:
	void act() override {
		if (enter) {
			std::cout << "Entering tracking state.\n";
			startTime = system_clock::now();
			controller_.reset();
			enter = false;
		}
		Eigen::Vector3d vec = controller_.getOutput(drone_->getCurrentPosition(), {0.0,0.0,-6.0});
		drone_->goTo(0.0,0.0,vec[2] - 5.0);
	}
	bool to_stateLanding() {
		return (getDuration() > 15);
	}
	stateTracking(Drone* drone, MultiAxisPIDController& controller, Trajectory trajectory) :
		drone_{drone}, controller_{controller}, trajectory_{trajectory}, enter{true} {

	}
	float getDuration() {
	    system_clock::time_point currentTime = system_clock::now();
    	duration<float> duration = currentTime - startTime;
    	return duration.count();
	}

};

class stateLanding : public State
{
private:
	Drone* drone_;
public:
	void act() override {
		drone_->disarm();
	} 
	bool to_stateEnd() {
		return true;
	}
	stateLanding(Drone* drone) : drone_{drone} {}
};

class stateEnd : public State
{
private:
public:
	void act() override {
		std::cout << "End state!\n";
	}
	stateEnd() {}
};

class stepFSM : public FSM 
{
private:
	stateTakeOff takeOff;
	stateTracking tracking;
	stateLanding landing;
	stateEnd end;
public:
	bool isFinished() {
		return (returnState() == &end);
	}

	stepFSM(stateTakeOff takeOff, stateTracking tracking, stateLanding landing, stateEnd end) :
		FSM(&(this->takeOff)), takeOff{takeOff}, tracking{tracking}, landing{landing}, end{end}
	{
		EDGE(TakeOff, takeOff, Tracking, tracking);
		EDGE(Tracking, tracking, Landing, landing);
		EDGE(Landing, landing, End, end);
	}
};



int main(int argc, char *argv[])
{
	char loop;

	while (std::cout << "'y' to simulate autonomous landing.\n" && std::cin >> loop && loop == 'y') {

		float mean, stddev;

		std::cout << "Mean: ";
		std::cin >> mean;
		std::cout << "Standard deviation: ";
		std::cin >> stddev;

		
		rclcpp::init(argc, argv);
		auto drone = std::make_shared<Drone>();

		Eigen::Vector3d verticalGains = {1.5, 0.3, 0.3};
		Eigen::Vector3d horizontalGains = {1.5, 0.3, 0.3};

		MultiAxisPIDController controller(horizontalGains,verticalGains);
		Trajectory trajectory(6.0, 0.4, {8.0, 8.0}, mean, stddev);

		stateTakeOff takeoff(drone.get());
		stateTracking tracking(drone.get(), controller, trajectory);
		stateLanding landing(drone.get());
		stateEnd end;

		stepFSM myFSM(takeoff, tracking, landing, end);

		while (rclcpp::ok() && !myFSM.isFinished())
		{
			myFSM.executeFSM();
			rclcpp:spin_some(drone);
		}
		rclcpp::shutdown();
		
	}	

	return 0;
}