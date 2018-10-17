#include <iostream>
#include <string>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco/charuco.hpp>
#include "util.hpp"

#include "camerasensor.hpp"
#include "calibration.hpp"
#include <franka/robot.h>
#include <franka/model.h>
#include <eigen3/Eigen/Core>

////////////////////////////////////////////////////////////////////////////////
#define ROBOT_IP "10.0.0.1"

////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv) {

	std::string resource_path = "/home/mrgribbot/calib-dataset2/";
	std::string mkdir_cmd = "mkrid " + resource_path;
	system(mkdir_cmd.c_str());

	CameraSensor camera;

	cv::Ptr<cv::aruco::Dictionary> dictionary = 
		cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

	Calibration calib(dictionary);

	uint32_t i = 0;

	franka::Robot robot(ROBOT_IP);

	while (true) {

		cv::Mat image_copy, image;
		cv::cvtColor(image, image_copy, cv::COLOR_GRAY2RGB);
		cv::Mat board_pose = calib.estimateCharucoPose(image_copy, &camera);

		if (!board_pose.empty()) {

			cv::imwrite(resource_path + "calib-images/ir" + std::to_string(i) + ".png", image);

			Util::writeToFile(resource_path + "cTch.csv", board_pose, i);

			robot.read([resource_path, i](const franka::RobotState& robot_state) {

				std::array<double, 16> pose = robot_state.O_T_EE;

				cv::Mat endeff_pose(cv::Size(4, 4), CV_64FC1);

				for (uint8_t j = 0; j < 4; j++) {

					for (uint8_t k = 0; k < 4; k++) {

						endeff_pose.at<double>(j, k) = pose[j * 4 + k];
					}
				}

				Util::writeToFile(resource_path + "bTe.csv", endeff_pose, i);
				return 0;
			});

			i++;
		}

	}
}

/// @file

