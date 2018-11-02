#include <iostream>
#include <opencv2/core/core.hpp>

#include "util.hpp"
#include "calibration.hpp"

////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv) {

	std::string resource_path_prefix =
		"/home/mrgribbot/Documents/calib-dataset1/";
	
	std::string local_work_dir =
		"/home/mrgribbot/catkin_ws/src/camera_calibration_panda/";

	std::string local_res = local_work_dir + "res/";

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

				Util::writeToFile(local_res + "cTch_ver.csv", board_pose, i);
			}
		}
	}

	std::vector<int> bTe_indices;
	std::vector<cv::Mat> bTe  = Util::readPosesFromFile(
		resource_path_prefix + "bTe.csv", &bTe_indices);

	std::vector<int> cTch_indices;
	std::vector<cv::Mat> cTch = Util::readPosesFromFile(local_res +
		"cTch_ver.csv", &cTch_indices);

	std::string compute_camera_pose_cmd = "octave " + local_work_dir +
		"src/estimate_camera_pose.m";

	system(compute_camera_pose_cmd.c_str());

	std::vector<cv::Mat> bTc = Util::readPosesFromFile(local_res + 
		"bTc.csv", NULL);

	std::vector<cv::Mat> eTch = Calibration::computeEndeffToCharuco(&bTe,
		&bTe_indices, &cTch, &cTch_indices, bTc[0]);

	for (uint32_t j = 0; j < eTch.size(); j++)
		Util::writeToFile(local_res + "eTch.csv", eTch[j], j);

	std::string variance_cmd = "python3 " + local_work_dir + "src/avg_quat.py";
	system(variance_cmd.c_str());

	return 0;
}

/// @file

