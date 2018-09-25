#include <iostream>
#include <string>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco/charuco.hpp>

#include <camerasensor.hpp>
#include <calibration.hpp>
#include <common/utility.hpp>

////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv) {

	CameraSensor camera;

	cv::Ptr<cv::aruco::Dictionary> dictionary = 
		cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

	Calibration calib(dictionary);

	for (uint32_t i = 0; i <= 15; i++) {

		std::string img_file = "ir" + std::to_string(i) + ".png";
		cv::Mat img = cv::imread("res/calib_images/" + img_file, cv::IMREAD_GRAYSCALE);
		cv::Mat img_copy;
		cv::cvtColor(img, img_copy, cv::COLOR_GRAY2RGB);
		cv::Mat board_pose = calib.EstimateCharucoPose(img_copy, &camera);

		if (!board_pose.empty()) {

			Util::write_to_file("res/board_poses_corrected.csv", board_pose, i);
		}
	}

	return 0;
}

