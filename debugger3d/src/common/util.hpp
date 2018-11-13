#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <string>

////////////////////////////////////////////////////////////////////////////////
namespace Util {

    bool validXYZ(double x, double y, double z);

    bool getUserPosition(double* x, double* y, double* z, bool* exit);

    Eigen::Quaterniond UniformRandomQuat();

	void writeToFile(std::string filepath, cv::Mat board_pose, int pose_idx);

	std::string poseToString(cv::Mat pose);

	void printCvMat(cv::Mat mat);

	std::vector<cv::Mat> readPosesFromFile(const std::string filename,
		std::vector<int>* indices);

	cv::Mat lineToPose(std::string line);

	/**
	 * @brief Splits the string at the given delimiter and returns
	 * a vector of floats
	 */
	std::vector<float> splitf(std::string str, std::string delim);
};

/// @file

