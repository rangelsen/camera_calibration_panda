
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <string>

namespace Util {

    bool validXYZ(double x, double y, double z);

    bool getUserPosition(double* x, double* y, double* z);

    Eigen::Quaterniond UniformRandomQuat();

	void writeToFile(std::string filepath, cv::Mat board_pose, int pose_idx);

	std::string poseToString(cv::Mat pose);
};

/// @file

