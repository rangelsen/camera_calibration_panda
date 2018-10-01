#include <queue>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <iomanip>

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

////////////////////////////////////////////////////////////////////////////////
void Util::printCvMat(cv::Mat mat) {

	std::cout << "cv::Mat" << std::endl;

	for (uint32_t i = 0; i < mat.rows; i++) {

		for (uint32_t j = 0; j < mat.cols; j++)
			std::cout << std::setprecision(6) << mat.at<double>(i, j) << " ";

		std::cout << std::endl;
	}

	std::cout << std::endl;
}

////////////////////////////////////////////////////////////////////////////////
std::vector<cv::Mat> Util::readPosesFromFile(const std::string filename,
	std::vector<int>* indices) {

	std::ifstream file(filename);
	std::string raw_line;

	std::vector<cv::Mat> poses;

	while(std::getline(file, raw_line)) {

		int i = 0;
		while (raw_line[i++] != ',') ;

		int index = std::atoi(raw_line.substr(0, i).c_str());

		if (indices)
			indices->push_back(index);

		cv::Mat pose = lineToPose(raw_line.substr(i, raw_line.length()));
		poses.push_back(pose);
	}

	return poses;
}

////////////////////////////////////////////////////////////////////////////////
cv::Mat Util::lineToPose(std::string line) {

	std::vector<float> vals = splitf(line, ",");

	cv::Mat pose(cv::Size(4, 4), CV_64FC1);

	for (uint8_t i = 0; i < pose.rows; i++)
		for (uint8_t j = 0; j < pose.cols; j++)
			pose.at<double>(i, j) = vals[i * 4 + j];

	return pose;
}

////////////////////////////////////////////////////////////////////////////////
std::vector<float> Util::splitf(std::string str, std::string delim) {

	std::vector<float> vals;
	uint32_t search_idx = 0;
	bool is_first = true;
	uint32_t n_entries = 0;

	while (n_entries < 16) {

		uint32_t start_cell;

		if (is_first) {

			start_cell = 0;
			is_first = false;
		}
		else
			start_cell = str.find(delim, search_idx) + 1;

		uint32_t end_cell = str.find(delim, start_cell + 1);
		uint32_t len = end_cell - start_cell;

		std::string cell = str.substr(start_cell, len);
		float val = std::stof(cell);

		vals.push_back(val);

		search_idx = end_cell;
		n_entries++;
	}

	return vals;
}

/// @file

