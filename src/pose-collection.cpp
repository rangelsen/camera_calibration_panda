#include <iostream>
#include <string>
#include <fstream>
#include <franka/robot.h>
#include <franka/model.h>
#include <stdio.h>
#include "util.hpp"

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
int main(int argc, char** argv) {

	if (argc < 4) {

		std::cout << "Not enough input arguments." << std::endl;
		std::cout << "Usage: collect-pose <storage-path> <robot-ip> <pose-index>"
				  << std::endl;

		exit(-1);
	}

	std::string storage_path = argv[1];
	std::string robot_ip = argv[2];
	uint32_t idx = std::atoi(argv[3]);

	franka::Robot robot(robot_ip);

	robot.read([storage_path, idx](const franka::RobotState& robot_state) {

		std::array<double, 16> pose = robot_state.O_T_EE;
		cv::Mat endeff_pose(cv::Size(4, 4), CV_64FC1);

		for (uint8_t j = 0; j < 4; j++) {

			for (uint8_t k = 0; k < 4; k++)
				endeff_pose.at<double>(k, j) = pose[j * 4 + k];
		}

		Util::writeToFile(storage_path + "bTe.csv", endeff_pose, idx);

		return 0;
	});

	return 0;
}


