#include <iostream>
#include <opencv2/core/core.hpp>

#include "util.hpp"
#include "calibration.hpp"

////////////////////////////////////////////////////////////////////////////////
static const std::vector<std::string> CAMERA_SERIALS {

	"810512060827",
	"747612060748",
};

////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv) {

	std::string local_work_dir =
		"/home/mrgribbot/catkin_ws/src/camera_calibration_panda/";

	std::string local_res = local_work_dir + "res/";
	std::string resource_path_prefix = local_res + "calib-dataset3/";
	
	/*
	{ // Index board poses that are visible from the camera

		// Remove old poses
		std::string rm_cTch = "rm " + local_res + "cTch_ver.csv";
		std::string rm_eTch = "rm " + local_res + "eTch.csv";

		system(rm_cTch.c_str());
		system(rm_eTch.c_str());

		CameraSensor camera;

		cv::Ptr<cv::aruco::Dictionary> dictionary = 
			cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

		Calibration calib(dictionary);

		for (uint32_t i = 0; i <= 15; i++) {

			std::string img_file = "ir" + std::to_string(i) + ".png";
			cv::Mat img = cv::imread(resource_path_prefix + "calib_images/" +
				img_file, cv::IMREAD_GRAYSCALE);

			cv::Mat img_copy;
			cv::cvtColor(img, img_copy, cv::COLOR_GRAY2RGB);
			cv::Mat board_pose = calib.estimateCharucoPose(img_copy, &camera);

			if (!board_pose.empty()) {

				Util::writeToFile(resource_path_prefix + "cTch_ver.csv", board_pose, i);
			}
		}
	}
	*/

	std::vector<int> bTe_indices;
	std::vector<cv::Mat> bTe  = Util::readPosesFromFile(
		resource_path_prefix + "bTe.csv", &bTe_indices);

	for (uint16_t i = 0; i < CAMERA_SERIALS.size(); i++) {

		std::vector<int> cTch_indices;
		std::vector<cv::Mat> cTch = Util::readPosesFromFile(resource_path_prefix +
			"cTch_" + CAMERA_SERIALS[i] + ".csv", &cTch_indices);

		std::cout << "read " << cTch.size() << " poses for camera "
			<< CAMERA_SERIALS[i] << std::endl;

		std::vector<cv::Mat> bTc = Util::readPosesFromFile(
			resource_path_prefix + "bTc_" + CAMERA_SERIALS[i] + ".csv", NULL);

		std::vector<cv::Mat> eTch = Calibration::computeEndeffToCharuco(&bTe,
			&bTe_indices, &cTch, &cTch_indices, bTc[0]);

		for (uint32_t j = 0; j < eTch.size(); j++)
			Util::writeToFile(local_res + "eTch.csv", eTch[j], j);

		std::string variance_cmd = "python3 " + local_work_dir + "src/avg_quat.py";
		system(variance_cmd.c_str());
	}

	return 0;
}

/// @file

