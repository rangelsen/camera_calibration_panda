#ifndef CALIB_CONFIG_HPP
#define CALIB_CONFIG_HPP

#include <string>

////////////////////////////////////////////////////////////////////////////////
class CalibConfig {

public:
	CalibConfig();

	CalibConfig(std::string file);

	std::map<std::string, cv::Mat poses> camera_poses;
};

#endif // CALIB_CONFIG_HPP

/// @file

