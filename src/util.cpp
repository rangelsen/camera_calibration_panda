#include <queue>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>

#include "util.hpp"

////////////////////////////////////////////////////////////////////////////////
#define MIN_X 0.0
#define MAX_X 0.7
#define MIN_Y -0.7
#define MAX_Y 0.7
#define MIN_Z 0.0
#define MAX_Z 1.0

////////////////////////////////////////////////////////////////////////////////
bool Util::validXYZ(double x, double y, double z) {

	bool input_x_valid = (x >= MIN_X && x <= MAX_X);
	bool input_y_valid = (y >= MIN_Y && y <= MAX_Y);
	bool input_z_valid = (z >= MIN_Z && z <= MAX_Z);
	return (input_x_valid && input_y_valid && input_z_valid);
}

////////////////////////////////////////////////////////////////////////////////
bool Util::getUserPosition(double* x, double* y, double* z) {

    std::cout << "Input: " << std::endl;

	std::string input;
	std::string command, tmp;

    std::queue<std::string> v;

    std::getline(std::cin, input);
    std::istringstream iss(input);
    std::getline(iss, command, ' ');

    if (command == "pos") {

        while (std::getline(iss, tmp, ' ')) {

            v.push(tmp);
        }

        if (v.size() != 3) {

            std::cout << "Syntax error: 3 arguments expected" << std::endl;
            return false;
        }

        *x = std::atof(v.front().c_str());
        v.pop();
        *y = std::atof(v.front().c_str());
        v.pop();
        *z = std::atof(v.front().c_str());
        v.pop();

        if (!validXYZ(*x, *y, *z)) {

            std::cout << "Setpoint out of bounds" << std::endl;
            return false;
        }

        return true;
    }
}

////////////////////////////////////////////////////////////////////////////////
Eigen::Quaterniond Util::UniformRandomQuat() {

    double u1 = (double) rand() / RAND_MAX;
    double u2 = (double) rand() / RAND_MAX;
    double u3 = (double) rand() / RAND_MAX;

    Eigen::Quaterniond quat;

    quat.w() = sqrt(1.0 - u1) * sinf(M_2_PI * u2);
    quat.x() = sqrt(1.0 - u1) * cosf(M_2_PI * u2);
    quat.y() = sqrt(u1) * sinf(M_2_PI * u3);
    quat.z() = sqrt(u1) * cosf(M_2_PI * u3);

    return quat;
}

////////////////////////////////////////////////////////////////////////////////
std::string Util::poseToString(cv::Mat pose) {
	
	std::string pose_str;

	for (uint8_t i = 0; i < 4; i++) {

		for (uint8_t j = 0; j < 4; j++)  {

			pose_str += std::to_string(pose.at<double>(i, j));

			if (!(i == 3 && j == 3))
				pose_str += ", ";
		}
	}

	return pose_str;
}

////////////////////////////////////////////////////////////////////////////////
void Util::writeToFile(std::string filepath, cv::Mat board_pose, int pose_idx) {

	std::string pose_str = std::to_string(pose_idx) + ", " + poseToString(board_pose);
	std::ofstream file;
	file.open(filepath, std::ios::out | std::ios::app);
	file << pose_str << std::endl;
	file.close();
}

/// @file

