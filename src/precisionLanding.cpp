#include "Drone.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <vector>
#include <tuple>
#include <fstream>


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
	std::ofstream &csvDrone_, &csvBaseTruth_, &csvBaseMeasure_;
	bool enter;
	int counter_;
public:
	void act() override {
		if (enter) {
			RCLCPP_INFO(drone_->get_logger(), "Entering tracking state.");
			controller_.reset();
			enter = false;
		}

		auto [xMeasure,yMeasure,xTruth,yTruth] = trajectory_.getPoint();
		auto pos = drone_->getCurrentPosition();
		Eigen::Vector3d vec = controller_.getOutput(pos, {xMeasure,yMeasure,-1.0});

		csvDrone_ << pos[0] << "," << pos[1] << "," << pos[2] << std::endl;
		csvBaseTruth_ << xTruth << "," << yTruth << "," << 0.0 << std::endl;
		csvBaseMeasure_ << xMeasure << "," << yMeasure << "," << 0.0 << std::endl;

		drone_->goTo(vec[0],vec[1],vec[2]);
	}
	bool to_stateApproaching() {
		auto [x,y,d,dd] = trajectory_.getPoint();
		Eigen::Vector3d diff = Eigen::Vector3d({x,y,-1.0}) - drone_->getCurrentPosition();
		csvDrone_ << diff.norm() << ",";
		if (diff.norm() < 0.20)
			counter_ ++;
		return (counter_ > 5);
	}

	stateTracking(Drone* drone, MultiAxisPIDController& controller, Trajectory trajectory, 
					std::ofstream &csvDrone, std::ofstream &csvBaseTruth, std::ofstream &csvBaseMeasure) :
		drone_{drone}, 
		controller_{controller}, 
		trajectory_{trajectory}, 
		csvDrone_{csvDrone},
		csvBaseTruth_{csvBaseTruth},
		csvBaseMeasure_{csvBaseMeasure},
		enter{true}, 
		counter_(0) 
	{
		csvDrone_ << "Norm,X,Y,Z" << std::endl;
		csvBaseTruth_ << "X,Y,Z" << std::endl;
		csvBaseMeasure_ << "X,Y,Z" << std::endl;
	}
};

class stateApproaching : public State
{
private:
	Drone* drone_;
	MultiAxisPIDController controller_;
	Trajectory trajectory_;
	std::ofstream &csvDrone_, &csvBaseTruth_, &csvBaseMeasure_;
	bool enter;
	int counter_;
public:
	void act() override {
		if (enter) {
			RCLCPP_INFO(drone_->get_logger(), "Entering approaching state.");
			enter = false;
		}

		auto [xMeasure,yMeasure,xTruth,yTruth] = trajectory_.getPoint();
		auto pos = drone_->getCurrentPosition();
		Eigen::Vector3d vec = controller_.getOutput(pos, {xMeasure,yMeasure,-0.25});

		csvDrone_ << pos[0] << "," << pos[1] << "," << pos[2] << "," << vec[2]<<std::endl;
		csvBaseTruth_ << xTruth << "," << yTruth << "," << 0.0 << std::endl;
		csvBaseMeasure_ << xMeasure << "," << yMeasure << "," << 0.0 << std::endl;

		drone_->goTo(vec[0],vec[1],vec[2]);
	}
	bool to_stateLanding() {
		auto [x,y,d,dd] = trajectory_.getPoint();
		Eigen::Vector3d diff = Eigen::Vector3d({x,y,-0.25}) - drone_->getCurrentPosition();
		if (diff.norm() < 0.20)
			counter_ ++;
		return (counter_ > 5);
	}

	stateApproaching(Drone* drone, MultiAxisPIDController& controller, Trajectory trajectory, 
					std::ofstream &csvDrone, std::ofstream &csvBaseTruth, std::ofstream &csvBaseMeasure) :
		drone_{drone}, 
		controller_{controller}, 
		trajectory_{trajectory}, 
		csvDrone_{csvDrone},
		csvBaseTruth_{csvBaseTruth},
		csvBaseMeasure_{csvBaseMeasure},
		enter{true}, 
		counter_(0) 
	{}
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
	stateApproaching approaching;
	stateEnd end;
public:
	bool isFinished() {
		return (returnState() == &end);
	}

	stepFSM(stateTakeOff takeOff, stateTracking tracking, stateLanding landing, stateApproaching approaching, stateEnd end) :
		FSM(&(this->takeOff)), takeOff{takeOff}, tracking{tracking}, landing{landing}, approaching{approaching}, end{end}
	{
		EDGE(TakeOff, takeOff, Tracking, tracking);
		EDGE(Tracking, tracking, Approaching, approaching);
		EDGE(Approaching, approaching, Landing, landing);
		EDGE(Landing, landing, End, end);
	}
};



int main(int argc, char *argv[])
{
	char loop;

	while (std::cout << "'y' to simulate autonomous landing.\n" && std::cin >> loop && loop == 'y') {

		float mean = 0.0, stddev;

		//std::cout << "Mean: ";
		//std::cin >> mean;
		std::cout << "Standard deviation: ";
		std::cin >> stddev;

		std::string csvDroneFilename = "results/data/landing/drone/DRONE__" + std::to_string(stddev) + ".csv";
		std::string csvBaseTruthFilename = "results/data/landing/truth/BASE_TRUTH__" + std::to_string(stddev) + ".csv";
		std::string csvBaseMeasureFilename = "results/data/landing/measured/BASE_MEASURED__" + std::to_string(stddev) + ".csv";
		
		std::ofstream csvDrone(csvDroneFilename);
		std::ofstream csvBaseTruth(csvBaseTruthFilename);
		std::ofstream csvBaseMeasure(csvBaseMeasureFilename);

		
		rclcpp::init(argc, argv);
		rclcpp::Rate loop_rate(60);
		auto drone = std::make_shared<Drone>();

		Eigen::Vector3d verticalGains = {6.9, 11.5, 1.035};
		Eigen::Vector3d horizontalGains = {1.35, 1.08, 0.42};

		MultiAxisPIDController controller(horizontalGains,verticalGains);
		controller.setWindup({-10.0, 10.0},{-10.0, 0.0});

		Trajectory trajectory(6.0, 0.4, {8.0, 8.0}, mean, stddev);

		stateTakeOff takeoff(drone.get());
		stateTracking tracking(drone.get(), controller, trajectory, csvDrone, csvBaseTruth, csvBaseMeasure);
		stateApproaching approaching(drone.get(), controller, trajectory, csvDrone, csvBaseTruth, csvBaseMeasure);
		stateLanding landing(drone.get());
		stateEnd end;

		stepFSM myFSM(takeoff, tracking, landing, approaching, end);

		while (rclcpp::ok() && !myFSM.isFinished())
		{
			myFSM.executeFSM();
			rclcpp:spin_some(drone);
			loop_rate.sleep();
		}
		rclcpp::shutdown();
		
	}	

	return 0;
}