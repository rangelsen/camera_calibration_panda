#include <fstream>
#include "utility.hpp"

////////////////////////////////////////////////////////////////////////////////
std::string Util::pose_to_string(cv::Mat pose) {
	
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
void Util::write_to_file(std::string filepath, cv::Mat board_pose, int pose_idx) {

	std::string pose_str = std::to_string(pose_idx) + ", " + pose_to_string(board_pose);
	std::ofstream file;
	file.open(filepath, std::ios::out | std::ios::app);
	file << pose_str << std::endl;
	file.close();
}

/// @file

