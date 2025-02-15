#include <iostream>
#include "calibration.hpp"

////////////////////////////////////////////////////////////////////////////////
/*
#define BOARD_SQUARE_LEN 0.0205f
#define BOARD_MARKER_LEN 0.0155f
#define BOARD_N_MARKERS 58
#define BOARD_N_SQUARES_X 13
#define BOARD_N_SQUARES_Y 9
*/

/*
#define BOARD_SQUARE_LEN 0.093f
#define BOARD_MARKER_LEN 0.07f
#define BOARD_N_MARKERS 3
#define BOARD_N_SQUARES_X 3
#define BOARD_N_SQUARES_Y 2
*/

/*
#define BOARD_SQUARE_LEN 0.0565f
#define BOARD_MARKER_LEN 0.0425f
#define BOARD_N_MARKERS 7
#define BOARD_N_SQUARES_X 5
#define BOARD_N_SQUARES_Y 3
*/

#define BOARD_SQUARE_LEN 0.045f
#define BOARD_MARKER_LEN 0.034f
#define BOARD_N_MARKERS 12
#define BOARD_N_SQUARES_X 6
#define BOARD_N_SQUARES_Y 4

////////////////////////////////////////////////////////////////////////////////
Calibration::Calibration(cv::Ptr<cv::aruco::Dictionary> dict) {

	dict_ = dict;

	board_ = cv::aruco::CharucoBoard::create(BOARD_N_SQUARES_X,
		BOARD_N_SQUARES_Y, BOARD_SQUARE_LEN, BOARD_MARKER_LEN, dict_);
}

////////////////////////////////////////////////////////////////////////////////
cv::Mat Calibration::estimateCharucoPosePoints(cv::Mat& image, CameraSensor* camera) {

	std::vector<int> charuco_ids;
	std::vector<std::vector<cv::Point2f>> corners;
	cv::aruco::detectMarkers(image, dict_, corners, charuco_ids);

	float detection_ratio = (float) corners.size() / BOARD_N_MARKERS;

	std::cout << "marker detection ratio: " << detection_ratio << std::endl;

	std::vector<cv::Vec3d> rot, trans;
	cv::Mat board_pose;

	if (charuco_ids.size() > 0) {

		cv::aruco::drawDetectedMarkers(image, corners, charuco_ids);

		cv::Mat intrin = camera->Intrinsics("rgb");
		cv::Mat dist_coeffs = camera->DistCoeffs("rgb");

		cv::aruco::estimatePoseSingleMarkers(corners, BOARD_MARKER_LEN,
			intrin, dist_coeffs, rot, trans); 	

		for (uint32_t i = 0; i < corners.size(); i++) {
			
			cv::aruco::drawAxis(image, intrin,
				dist_coeffs, rot[i], trans[i], BOARD_MARKER_LEN);
		}
		
		cv::Vec3d board_trans = trans[0];
		cv::Vec3d board_rot = rot[0];

		cv::aruco::drawAxis(image, intrin, dist_coeffs,
			board_rot, board_trans, 5.0f * BOARD_MARKER_LEN);

		cv::Mat board_rot_mat;
		cv::Rodrigues(board_rot, board_rot_mat);

		board_pose = cv::Mat::zeros(cv::Size(4, 4), CV_64FC1);

		board_rot_mat.copyTo(board_pose(cv::Range(0, 3), cv::Range(0, 3)));

		for (uint8_t i = 0; i < 3; i++)
			board_pose.at<double>(i, 3) = board_trans[i];
	}

	return board_pose;
}

////////////////////////////////////////////////////////////////////////////////
cv::Mat Calibration::estimateCharucoPose(cv::Mat& image, CameraSensor* camera) {

	std::vector<int> charuco_ids, all_ids;
	std::vector<std::vector<cv::Point2f>> corners, rejected_markers;
	std::vector<cv::Point2f> all_corners;
	cv::Ptr<cv::aruco::DetectorParameters> detector_params;
	cv::aruco::detectMarkers(image, dict_, corners, charuco_ids);

	float detection_ratio = (float) corners.size() / BOARD_N_MARKERS;

	std::cout << "marker detection ratio: " << detection_ratio << std::endl;

	/*
	cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(
		BOARD_N_SQUARES_X, BOARD_N_SQUARES_Y, BOARD_SQUARE_LEN, BOARD_MARKER_LEN, dict_);
	*/

	cv::Mat intrin = camera->Intrinsics("rgb");
	cv::Mat dist_coeffs = camera->DistCoeffs("rgb");

	int interpolatedCorners = 0;

	if(charuco_ids.size() > 0) {
		
		interpolatedCorners = cv::aruco::interpolateCornersCharuco(
			corners, charuco_ids, image, board_, all_corners, all_ids, intrin,
			dist_coeffs);

		cv::aruco::drawDetectedMarkers(image, corners, charuco_ids);

	}

	cv::Mat board_pose;

	cv::Vec3d rot, trans;

	bool has_board_pose = cv::aruco::estimatePoseCharucoBoard(all_corners,
		all_ids, board_, intrin, dist_coeffs, rot, trans);

	if (!has_board_pose)
		return board_pose;

	cv::aruco::drawAxis(image, intrin, dist_coeffs,
		rot, trans, 5.0f * BOARD_MARKER_LEN);

	board_pose = cv::Mat::zeros(cv::Size(4, 4), CV_64FC1);
	cv::Mat board_rot_mat;
	cv::Rodrigues(rot, board_rot_mat);
	board_rot_mat.copyTo(board_pose(cv::Range(0, 3), cv::Range(0, 3)));

	for (uint8_t i = 0; i < 3; i++)
		board_pose.at<double>(i, 3) = trans[i];

	board_pose.at<double>(3, 3) = 1.0;

	return board_pose;
}

////////////////////////////////////////////////////////////////////////////////
std::vector<cv::Mat> Calibration::computeEndeffToCharuco(
	std::vector<cv::Mat>* bTe, std::vector<int>* bTe_indices,
	std::vector<cv::Mat>* cTch, std::vector<int>* cTch_indices, cv::Mat bTc) {

	std::vector<cv::Mat> eTch;

	for (uint32_t i = 0; i < bTe_indices->size(); i++) {

		for (uint32_t j = 0; j < cTch_indices->size(); j++) {
			
			if ((*cTch_indices)[j] == (*bTe_indices)[i]) {

				cv::Mat tf = (*bTe)[i].inv() * bTc * (*cTch)[j];
				eTch.push_back(tf);
			}
		}
	}

	return eTch;
}

/// @file

