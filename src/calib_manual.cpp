#include <iostream>
#include <string>
#include <fstream>
#include <unistd.h>
#include <cassert>
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

	std::string root_path = Util::getRootPath();

	assert(argc > 1);
	std::string resource_path = root_path + "res/" + std::string(argv[1]) + "/"; 

	std::string rm_cmd = "rm -rf " + resource_path;
	std::string mkdir_cmd = "mkdir " + resource_path;
	// system(rm_cmd.c_str());
	system(mkdir_cmd.c_str());

	cv::Ptr<cv::aruco::Dictionary> dictionary = 
		cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

	Calibration calib(dictionary);
	CameraSensor::Initialize();

	for (CameraSensor* camera : CameraSensor::connected_devices) {

		std::string intrin_string = Util::poseToString(camera->GetIntrinsics("depth"));

		std::ofstream file;
		file.open(resource_path + "camera_properties.csv", std::ios::app);
		file << camera->SerialNumber() + "," + std::to_string(camera->MeterScale()) +
			+ "," + intrin_string + "\n";
		file.close();

		std::string mkdir_ir_cmd = "mkdir " + resource_path +
			"calib-images-ir-" + camera->SerialNumber();
		std::string mkdir_depth_cmd = "mkdir " + resource_path +
			"calib-images-depth-" + camera->SerialNumber();
		std::string mkdir_rgb_cmd = "mkdir " + resource_path +
			"calib-images-rgb-" + camera->SerialNumber();
		std::string mkdir_rgb_cmd_raw = "mkdir " + resource_path +
			"calib-images-rgb-raw-" + camera->SerialNumber();

		system(mkdir_ir_cmd.c_str());
		system(mkdir_depth_cmd.c_str());
		system(mkdir_rgb_cmd.c_str());
		system(mkdir_rgb_cmd_raw.c_str());
	}

	uint32_t i = 0;

	while (true) {

		cv::Mat colorized_image, ir_image, depth_image, rgb_image;

		for (CameraSensor* camera : CameraSensor::connected_devices) {

			camera->CaptureIr(&ir_image);
			camera->CaptureDepth(&depth_image);
			camera->CaptureRgb(&rgb_image);

			cv::cvtColor(ir_image, colorized_image, cv::COLOR_GRAY2RGB);

			cv::Mat board_pose = calib.estimateCharucoPose(colorized_image, camera);

			if (!board_pose.empty()) {

				std::cout << "camera " << camera->SerialNumber() << " has board pose" << std::endl;
				cv::imwrite(resource_path + "calib-images-ir-" + camera->SerialNumber() +
					"/ir" + std::to_string(i) + ".png", ir_image);

				cv::imwrite(resource_path + "calib-images-depth-" + camera->SerialNumber() +
					"/depth" + std::to_string(i) + ".png", depth_image);

				cv::imwrite(resource_path + "calib-images-rgb-" + camera->SerialNumber() +
					"/rgb" + std::to_string(i) + ".png", colorized_image);

				cv::imwrite(resource_path + "calib-images-rgb-raw-" + camera->SerialNumber() +
					"/rgb" + std::to_string(i) + ".png", rgb_image);

				Util::writeToFile(resource_path + "cTch_" + camera->SerialNumber() + ".csv", board_pose, i);

				cv::imshow("Detection", colorized_image);
				cv::waitKey(0);
			}
		}

		std::string franka_cmd = root_path + "build/collect-pose " +
			resource_path + " " + ROBOT_IP + " " + std::to_string(i);

		system(franka_cmd.c_str());

		i++;

		std::cout << "Press any key to continue" << std::endl;
		cv::waitKey(0);

		/*
		std::cout << "Press any key to continue" << std::endl;
		std::cin.ignore();
		*/
	}
}

/// @file

