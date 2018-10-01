#include <iostream>
#include <opencv2/core/core.hpp>

#include "util.hpp"
#include "calibration.hpp"

////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv) {

	std::string resource_path_prefix =
		"/home/mrgribbot/Documents/camera-calibration/res/";

	std::cout << "Running calibration verification at:"
		<< resource_path_prefix << std::endl;

	std::vector<int> bTe_indices;
	std::vector<cv::Mat> bTe  = Util::readPosesFromFile(resource_path_prefix +
		"endeffector_poses.csv", &bTe_indices);

	std::vector<int> cTch_indices;
	std::vector<cv::Mat> cTch = Util::readPosesFromFile(resource_path_prefix +
		"board_poses_corrected.csv", &cTch_indices);

	std::vector<cv::Mat> bTc = Util::readPosesFromFile(resource_path_prefix +
		"camera_poses.csv", NULL);

	std::vector<cv::Mat> eTch = Calibration::computeEndeffToCharuco(&bTe,
		&bTe_indices, &cTch, &cTch_indices, bTc[0]);

	std::cout << "Verifying camera pose: " << std::endl;
	Util::printCvMat(bTc[0]);

	for (uint32_t j = 0; j < eTch.size(); j++) {
		
		Util::printCvMat(eTch[j]);
		Util::writeToFile("eTch.csv", eTch[j], j);
	}

	return 0;
}

/// @file

