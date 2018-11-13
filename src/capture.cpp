#include <iostream>
#include <string>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "util.hpp"

#include "camerasensor.hpp"
#include <eigen3/Eigen/Core>

////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv) {

	std::string resource_path =
		"/home/mrgribbot/catkin_ws/src/camera_calibration_panda/res/calib-dataset5/verification-defaul-camera-settings/";

	std::string mkdir_cmd = "mkdir " + resource_path;
	system(mkdir_cmd.c_str());

	CameraSensor::Initialize();

	for (CameraSensor* camera : CameraSensor::connected_devices) {

		std::string mkdir_ir_cmd = "mkdir " + resource_path +
			"ver-images-ir-" + camera->SerialNumber();
		std::string mkdir_depth_cmd = "mkdir " + resource_path +
			"ver-images-depth-" + camera->SerialNumber();
		std::string mkdir_rgb_cmd = "mkdir " + resource_path +
			"ver-images-rgb-" + camera->SerialNumber();
		std::string mkdir_rgb_cmd_raw = "mkdir " + resource_path +
			"ver-images-rgb-raw-" + camera->SerialNumber();

		system(mkdir_ir_cmd.c_str());
		system(mkdir_depth_cmd.c_str());
		system(mkdir_rgb_cmd.c_str());
		system(mkdir_rgb_cmd_raw.c_str());
	}

	uint32_t i = 0;

	while (true) {

		for (CameraSensor* camera : CameraSensor::connected_devices) {

			cv::Mat colorized_image, ir_image, depth_image, rgb_image;
			camera->CaptureIr(&ir_image);
			camera->CaptureDepth(&depth_image);
			camera->CaptureRgb(&rgb_image);

			cv::cvtColor(rgb_image, colorized_image, cv::COLOR_BGR2RGB);

			cv::imwrite(resource_path + "ver-images-ir-" + camera->SerialNumber() +
				"/ir" + std::to_string(i) + ".png", ir_image);

			cv::imwrite(resource_path + "ver-images-depth-" + camera->SerialNumber() +
				"/depth" + std::to_string(i) + ".png", depth_image);

			cv::imwrite(resource_path + "ver-images-rgb-" + camera->SerialNumber() +
				"/rgb" + std::to_string(i) + ".png", colorized_image);

			cv::imwrite(resource_path + "ver-images-rgb-raw-" + camera->SerialNumber() +
				"/rgb" + std::to_string(i) + ".png", rgb_image);

			cv::imshow("Capture", colorized_image);
			cv::waitKey(0);
		}

		i++;
	}
}

/// @file

