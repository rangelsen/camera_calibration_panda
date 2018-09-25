#include <iostream>
#include <calibration.hpp>

////////////////////////////////////////////////////////////////////////////////
#define BOARD_SQUARE_LEN 0.034f
#define BOARD_MARKER_LEN 0.026f
#define BOARD_N_MARKERS 20
#define BOARD_N_SQUARES_X 8
#define BOARD_N_SQUARES_Y 5

////////////////////////////////////////////////////////////////////////////////
Calibration::Calibration(cv::Ptr<cv::aruco::Dictionary> dict) {

	dict_ = dict;
}

////////////////////////////////////////////////////////////////////////////////
cv::Mat Calibration::EstimateCharucoPosePoints(cv::Mat& image, CameraSensor* camera) {

	std::vector<int> charuco_ids;
	std::vector<std::vector<cv::Point2f>> corners;
	cv::imshow("ir", image);
	cv::waitKey(0);
	cv::aruco::detectMarkers(image, dict_, corners, charuco_ids);

	float detection_ratio = (float) corners.size() / BOARD_N_MARKERS;

	std::cout << "marker detection ratio: " << detection_ratio << std::endl;

	std::vector<cv::Vec3d> rot, trans;
	cv::Mat board_pose;

	if (charuco_ids.size() > 0) {

		cv::aruco::drawDetectedMarkers(image, corners, charuco_ids);

		cv::Mat intrin = camera->Intrinsics("ir");
		cv::Mat dist_coeffs = camera->DistCoeffs("ir");

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

		cv::imshow("Detected markers", image);
		cv::waitKey(0);
	}

	return board_pose;
}

////////////////////////////////////////////////////////////////////////////////
cv::Mat Calibration::EstimateCharucoPose(cv::Mat& image, CameraSensor* camera) {

	std::vector<int> charuco_ids, all_ids;
	std::vector<std::vector<cv::Point2f>> corners;
	std::vector<cv::Point2f> all_corners;
	cv::imshow("ir", image);
	cv::waitKey(0);
	cv::aruco::detectMarkers(image, dict_, corners, charuco_ids);

	float detection_ratio = (float) corners.size() / BOARD_N_MARKERS;

	std::cout << "marker detection ratio: " << detection_ratio << std::endl;

	cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(
		BOARD_N_SQUARES_X, BOARD_N_SQUARES_Y, BOARD_SQUARE_LEN, BOARD_MARKER_LEN, dict_);

	cv::Mat intrin = camera->Intrinsics("ir");
	cv::Mat dist_coeffs = camera->DistCoeffs("ir");

	int interpolatedCorners = 0;

	if(charuco_ids.size() > 0) {
		
		interpolatedCorners = cv::aruco::interpolateCornersCharuco(
			corners, charuco_ids, image, board, all_corners, all_ids, intrin,
			dist_coeffs);
	}

	cv::Mat board_pose;

	cv::Vec3d rot, trans;

	bool has_board_pose = cv::aruco::estimatePoseCharucoBoard(all_corners,
		all_ids, board, intrin, dist_coeffs, rot, trans);

	if (!has_board_pose)
		return board_pose;

	cv::aruco::drawAxis(image, intrin, dist_coeffs,
		rot, trans, 5.0f * BOARD_MARKER_LEN);

	std::cout << "has board pose: " << has_board_pose << std::endl;

	board_pose = cv::Mat::zeros(cv::Size(4, 4), CV_64FC1);
	cv::Mat board_rot_mat;
	cv::Rodrigues(rot, board_rot_mat);
	board_rot_mat.copyTo(board_pose(cv::Range(0, 3), cv::Range(0, 3)));

	for (uint8_t i = 0; i < 3; i++)
		board_pose.at<double>(i, 3) = trans[i];

	board_pose.at<double>(3, 3) = 1.0;

	cv::imshow("Detected markers", image);
	cv::waitKey(0);

	return board_pose;
}

/// @file

