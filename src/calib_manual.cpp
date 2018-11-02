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
#define ROBOT_IP "10.0.0.2"

////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv) {

	std::string resource_path =
		"/home/mrgribbot/catkin_ws/src/camera_calibration_panda/res/calib-dataset1/";

	std::string rm_cmd = "rm -rf " + resource_path;
	std::string mkdir_cmd = "mkdir " + resource_path;
	std::string mkdir_ir_cmd = "mkdir " + resource_path + "calib-images-ir";
	std::string mkdir_depth_cmd = "mkdir " + resource_path + "calib-images-depth";
	system(rm_cmd.c_str());
	system(mkdir_cmd.c_str());
	system(mkdir_ir_cmd.c_str());
	system(mkdir_depth_cmd.c_str());

	cv::Ptr<cv::aruco::Dictionary> dictionary = 
		cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

	Calibration calib(dictionary);
	CameraSensor::Initialize();

	for (CameraSensor* camera : CameraSensor::connected_devices) {

		std::ofstream file;
		file.open(resource_path + "depth_scale.cfg", std::ios::app);
		file << camera->SerialNumber() + "," + std::to_string(camera->MeterScale());
		file.close();
	}

	uint32_t i = 0;

	while (true) {

		for (CameraSensor* camera : CameraSensor::connected_devices) {

			cv::Mat colorized_image, ir_image, depth_image;
			camera->CaptureIr(&ir_image);
			camera->CaptureDepth(&depth_image);

			cv::cvtColor(ir_image, colorized_image, cv::COLOR_GRAY2RGB);
			cv::Mat board_pose = calib.estimateCharucoPose(colorized_image, camera);

			if (!board_pose.empty()) {

				cv::imwrite(resource_path + "calib-images-ir/cam" + camera->SerialNumber() +
					"ir" + std::to_string(i) + ".png", ir_image);

				cv::imwrite(resource_path + "calib-images-depth/cam" + camera->SerialNumber() +
					"depth" + std::to_string(i) + ".png", depth_image);

				Util::writeToFile(resource_path + "cTch.csv", board_pose, i);

				std::string franka_cmd = "./devel/lib/camera_calibration_panda/collect-pose " +
					resource_path + " " + ROBOT_IP + " " + std::to_string(i);

				system(franka_cmd.c_str());

				i++;
			}

			cv::imshow("Detection", colorized_image);
			cv::waitKey(0);
		}
	}
}

/// @file

