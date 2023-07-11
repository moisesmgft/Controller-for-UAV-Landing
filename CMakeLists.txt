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
ament_target_dependencies(Drone px4_msgs rclcpp geometry_msgs sensor_msgs)
target_include_directories(Drone PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)

# Add Solution lib
add_library(Solution SHARED include/Solution.cpp)
target_link_libraries(Solution Drone)
target_include_directories(Solution PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)

# Add AutonomousDrone lib
add_library(AutonomousDrone SHARED include/AutonomousDrone.cpp)
target_link_libraries(AutonomousDrone Drone Trajectory)
target_include_directories(AutonomousDrone PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)

# Add Trajectory lib
add_library(Trajectory SHARED include/Trajectory.cpp)
target_link_libraries(Trajectory Drone)
target_include_directories(Trajectory PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)



add_executable(autonomous_landing src/autonomous_landing.cpp)
ament_target_dependencies(autonomous_landing rclcpp)
target_link_libraries(autonomous_landing Drone AutonomousDrone)
install(TARGETS autonomous_landing DESTINATION lib/${PROJECT_NAME})


############
# Install ##
############

# Export information to downstream packages
ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime eigen3_cmake_module Eigen3 px4_msgs geometry_msgs sensor_msgs)

ament_export_targets(export_Drone HAS_LIBRARY_TARGET)

ament_export_include_directories(include)
ament_export_libraries(Drone AutonomousDrone)

# Install header files
install(DIRECTORY include/ DESTINATION include)

install(TARGETS Drone
	EXPORT export_Drone
	INCLUDES DESTINATION include
)

install(TARGETS AutonomousDrone
	EXPORT export_AutonomousDrone
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