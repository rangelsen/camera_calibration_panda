#include <iostream>
#include <string>
#include <fstream>
#include <franka/robot.h>
#include <franka/model.h>
#include <eigen3/Eigen/Core>
#include <stdio.h>

////////////////////////////////////////////////////////////////////////////////
void print_pose(std::array<double, 16> pose) {

	printf("Endeffector pose: \n");

	for (uint8_t i = 0; i < 4; i++) {

		for (uint8_t j = 0; j < 4; j++) {

			printf("%.4f ", pose[j * 4 + i]);
		}
		printf("\n");
	}
}

////////////////////////////////////////////////////////////////////////////////
std::string pose_to_string(std::array<double, 16> pose) {
	
	std::string pose_str;

	for (uint8_t i = 0; i < 4; i++) {

		for (uint8_t j = 0; j < 4; j++)
			pose_str += std::to_string(pose[j * 4 + i]) + ", ";
	}

	return pose_str;
}

////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv) {

	if (argc < 3) {

		std::cout << "Not enough input arguments." << std::endl;
		std::cout << "Usage: collect-pose <storage-path> <robot-ip>" << std::endl;
		exit(-1);
	}

	std::string storage_path = argv[1];
	std::string robot_ip = argv[2];

	franka::Robot robot(robot_ip);

	robot.read([storage_path](const franka::RobotState& robot_state) {

		std::array<double, 16> pose = robot_state.O_T_EE;

		std::string pose_str = pose_to_string(pose);

		std::ofstream file;
		file.open(storage_path.c_str(), std::ios::out | std::ios::app);
		file << pose_str << std::endl;
		file.close();
		return 0;
	});

	return 0;
}


