#ifndef CALIBRATION_HPP
#define CALIBRATION_HPP

#include <opencv2/core/core.hpp>
#include <opencv2/aruco/charuco.hpp>

#include "camerasensor.hpp"

////////////////////////////////////////////////////////////////////////////////
class Calibration {

public:
	Calibration(cv::Ptr<cv::aruco::Dictionary> dict);

	/**
	 * @brief Estimate the position and orientation of the ChArUco board in the
	 * camera frame
	 */
	cv::Mat estimateCharucoPosePoints(cv::Mat& image, CameraSensor* camera);

	cv::Mat estimateCharucoPose(cv::Mat& image, CameraSensor* camera);

	static cv::Mat estimateBoardCorner(std::vector<int> ids,
		std::vector<cv::Vec3d> rot, std::vector<cv::Vec3d> trans);

	static std::vector<cv::Mat> computeEndeffToCharuco(
		std::vector<cv::Mat>* bTe, std::vector<cv::Mat>* cTch, cv::Mat bTc);

private:
	
	/**
	 * @brief ChArUco board on which calibration is performed
	 */
	cv::Ptr<cv::aruco::Dictionary> dict_;
};


#endif // CALIBRATION_HPP

/// @file

