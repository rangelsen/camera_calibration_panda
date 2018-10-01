#include <opencv2/core/core.hpp>

#include "util.hpp"

////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv) {

	std::string resource_path_prefix =
			"/home/mrgribbot/Documents/camera-calibration/res/";

		std::vector<int> bTe_indices;
		std::vector<cv::Mat> bTe  = Util::readPosesFromFile(resource_path_prefix +
			"endeff_poses.csv", &bTe_indices);

		std::vector<int> cTch_indices;
		std::vector<cv::Mat> cTch = Util::readPosesFromFile(resource_path_prefix +
			"board_poses.csv", &cTch_indices);

		std::vector<cv::Mat> bTc = Util::readPosesFromFile(resource_path_prefix +
			"camera_pose.csv", NULL);

		for (uint32_t j = 0; j < bTe.size(); j++) {

			Util::printCvMat(bTe[j]);
		}

		/*
		std::vector<cv::Mat> eTch = Calibration::computeEndeffToCharuco(&bTe, &cTch,
			bTc[0]);

		for (uint32_t j = 0; j < eTch.size(); j++) {
			
			Util::printCvMat(eTch[j]);
		}
		*/
}

/// @file

