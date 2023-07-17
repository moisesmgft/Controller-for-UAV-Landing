#include "Drone.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <vector>
#include <tuple>

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
	drone_->goTo(0.0,0.0,-3.0);
	//drone_->arm();
}
	bool to_stateStep() {
	Eigen::Vector3d diff = Eigen::Vector3d({0.0,0.0,-3.0}) - drone_->getCurrentPosition();
	return diff.norm() < 0.05;
}
	stateTakeOff(Drone* drone) : drone_(drone) {}
};

class stateStep : public State
{
private:
	Drone* drone_;
	MultiAxisPIDController controller_;
	system_clock::time_point startTime;
	bool enter;
public:
	void act() override {
		if (enter) {
			startTime = system_clock::now();
			controller_.setReference({1.0,0.0,-3.0});
			enter = false;
		}
		Eigen::Vector3d vec = controller_.getOutput(drone_->getCurrentPosition());
		drone_->goTo(vec[0],vec[1],vec[2]);
	}
	bool to_stateEnd() {
		return (getDuration() > 5);
	}
	stateStep(Drone* drone, MultiAxisPIDController& controller) :
		drone_{drone}, controller_{controller}, enter{true} {

	}
	float getDuration() {
	    system_clock::time_point currentTime = system_clock::now();
    	duration<float> duration = currentTime - startTime;
    	return duration.count();
	}

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
	stateStep step;
	stateEnd end;
public:
	bool isFinished() {
		return (returnState() == &end);
	}

	stepFSM(stateTakeOff takeOff, stateStep step, stateEnd end) : FSM(&(this->takeOff)), takeOff{takeOff}, step{step}, end{end}
	{
		EDGE(TakeOff, takeOff, Step, step);
		EDGE(Step, step, End, end);
	}
};



int main(int argc, char *argv[])
{
	/*
	char loop;

	while (std::cin >> loop && loop == 'y') {

		rclcpp::init(argc, argv);


		auto drone = std::make_shared<Drone>();

		rclcpp::spin(drone);

		rclcpp::shutdown();
		
	}
	*/

	rclcpp::init(argc, argv);


	auto drone = std::make_shared<Drone>();
	MultiAxisPIDController controller({1.0,1.0,1.0},{1.0,1.0,1.0});
	stateTakeOff takeoff(drone.get());
	stateStep step(drone.get(), controller);
	stateEnd end;

	stepFSM myFSM(takeoff,step,end);

	while (rclcpp::ok() && !myFSM.isFinished())
	{
		myFSM.executeFSM();
		rclcpp:spin_some(drone);
	}
	rclcpp::shutdown();

	

	return 0;
}