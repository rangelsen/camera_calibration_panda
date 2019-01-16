#ifndef UTILITY_HPP
#define UTILITY_HPP

#include <opencv2/core/core.hpp>
#include <string>

namespace Util {

	void write_to_file(std::string filepath, cv::Mat board_pose, int pose_idx);

	std::string pose_to_string(cv::Mat pose);
}

#endif // UTILITY_HPP

/// @file

