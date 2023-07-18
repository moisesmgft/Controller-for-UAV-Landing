#include "Drone.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <vector>
#include <tuple>
#include <fstream>

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
	bool to_stateStep() {
	Eigen::Vector3d diff = Eigen::Vector3d({0.0,0.0,-5.0}) - drone_->getCurrentPosition();
	return diff.norm() < 0.05;
}
	stateTakeOff(Drone* drone) : drone_(drone) {}
};

class stateStep : public State
{
private:
	Drone* drone_;
	MultiAxisPIDController controller_;
	std::ofstream &csvFile_;
	system_clock::time_point startTime;
	bool enter;
public:
	void act() override {
		if (enter) {
			RCLCPP_INFO(drone_->get_logger(), "Entering step state.");
			startTime = system_clock::now();
			controller_.reset();
			enter = false;
		}


		float time = getDuration();
		auto pos = drone_->getCurrentPosition();
		Eigen::Vector3d vec = controller_.getOutput(pos, {0.0,0.0,-6.0});

		csvFile_ << time << "," 
				 << pos[2] << "," 
				 << vec[2] << std::endl;

		drone_->goTo(0.0,0.0,vec[2] - 5.0);
	}
	bool to_stateEnd() {
		return (getDuration() > 15);
	}
	stateStep(Drone* drone, MultiAxisPIDController& controller, std::ofstream &csvFile) :
		drone_{drone}, controller_{controller}, csvFile_{csvFile}, enter{true} 
	{
		csvFile_ << "Time,Pos_Z,Output_Z" << std::endl;
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
		std::cout << "End state.\n";
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
	char loop;

	while (std::cout << "'y' to tune PID controller.\n" && std::cin >> loop && loop == 'y') {

		float vP, vI, vD;

		std::cout << "Verical gains: ";
		std::cin >> vP >> vI >> vD;

		std::string csvFilename = "results/data/vertical/VERT_" + std::to_string(vP) 
										   + "__" + std::to_string(vI) 
										   + "__" + std::to_string(vD) 
										   + ".csv";
		std::ofstream csvFile(csvFilename);
		
		rclcpp::init(argc, argv);
		rclcpp::Rate loop_rate(60);
		auto drone = std::make_shared<Drone>();

		Eigen::Vector3d verticalGains = {vP, vI, vD};
		Eigen::Vector3d horizontalGains = {0.0, 0.0, 0.0};

		MultiAxisPIDController controller(horizontalGains,verticalGains);
		controller.setWindup({0.0, 8.0},{-10.0, 10.0});


		stateTakeOff takeoff(drone.get());
		stateStep step(drone.get(), controller, csvFile);
		stateEnd end;

		stepFSM myFSM(takeoff,step,end);

		while (rclcpp::ok() && !myFSM.isFinished())
		{
			myFSM.executeFSM();
			rclcpp:spin_some(drone);
			loop_rate.sleep();
		}
		csvFile.close();
		rclcpp::shutdown();
		
		
	}

	

	return 0;
}