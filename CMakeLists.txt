cmake_minimum_required(VERSION 3.5)
project(pid_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

#################
# Setup targets #
#################

include_directories(include)

# Add Drone lib
add_library(Drone SHARED include/Drone.cpp)
ament_target_dependencies(Drone px4_msgs rclcpp geometry_msgs sensor_msgs Eigen3)
target_include_directories(Drone PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)




# Add Trajectory lib
add_library(Trajectory SHARED include/Trajectory.cpp)
target_include_directories(Trajectory PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)

# Add PIDController lib
add_library(PIDController SHARED include/PIDController.cpp)
ament_target_dependencies(PIDController Eigen3)
target_include_directories(PIDController PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)

# Add MultiAxisPIDController lib
add_library(MultiAxisPIDController SHARED include/MultiAxisPIDController.cpp)
ament_target_dependencies(MultiAxisPIDController Eigen3)
target_link_libraries(MultiAxisPIDController PIDController)
target_include_directories(MultiAxisPIDController PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)

# Add State lib
add_library(State SHARED include/State.cpp)
target_include_directories(State PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)

# Add FSM lib
add_library(FSM SHARED include/FSM.cpp)
target_link_libraries(FSM State)
target_include_directories(FSM PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)

add_executable(horizontalStepResponse src/horizontalStepResponse.cpp)
ament_target_dependencies(horizontalStepResponse rclcpp)
target_link_libraries(horizontalStepResponse Drone MultiAxisPIDController FSM State)
install(TARGETS horizontalStepResponse DESTINATION lib/${PROJECT_NAME})

add_executable(verticalStepResponse src/verticalStepResponse.cpp)
ament_target_dependencies(verticalStepResponse rclcpp)
target_link_libraries(verticalStepResponse Drone MultiAxisPIDController FSM State)
install(TARGETS verticalStepResponse DESTINATION lib/${PROJECT_NAME})

add_executable(precisionLanding src/precisionLanding.cpp)
ament_target_dependencies(precisionLanding rclcpp)
target_link_libraries(precisionLanding Drone MultiAxisPIDController FSM State Trajectory)
install(TARGETS precisionLanding DESTINATION lib/${PROJECT_NAME})


############
# Install ##
############

# Export information to downstream packages
ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime eigen3_cmake_module Eigen3 px4_msgs geometry_msgs sensor_msgs)

ament_export_targets(export_Drone HAS_LIBRARY_TARGET)

ament_export_include_directories(include)
ament_export_libraries(Drone  FSM)

# Install header files
install(DIRECTORY include/ DESTINATION include)

install(
	TARGETS Drone PIDController MultiAxisPIDController FSM State
	EXPORT export_Drone
	INCLUDES DESTINATION include
)


install(TARGETS Trajectory
	EXPORT export_Trajectory
	INCLUDES DESTINATION include
)


# Install launch files.
#install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# Install tests
#install(DIRECTORY test DESTINATION share/${PROJECT_NAME}/)


############
# Testing ##
############

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	# the following line skips the linter which checks for copyrights
	# uncomment the line when a copyright and license is not present in all source files
	#set(ament_cmake_copyright_FOUND TRUE)
	# the following line skips cpplint (only works in a git repo)
	# uncomment the line when this package is not in a git repo
	#set(ament_cmake_cpplint_FOUND TRUE)
	ament_lint_auto_find_test_dependencies()
endif()

ament_package()
