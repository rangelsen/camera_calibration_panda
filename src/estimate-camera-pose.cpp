#include <visp/vpCalibration.h>
#include <opencv2/core/core.hpp>
#include <vector>
#include <string>
#include <iostream>

////////////////////////////////////////////////////////////////////////////////
std::vector<float> split_float(std::string str, std::string delim) {

	std::vector<float> vals;
	uint32_t search_idx = 0;

	bool is_first = true;

	uint32_t n_entries = 0;

	while (n_entries < 16) {

		uint32_t start_cell;

		if (is_first) {

			start_cell = 0;
			is_first = false;
		}
		else
			start_cell = str.find(delim, search_idx) + 1;

		uint32_t end_cell = str.find(delim, start_cell + 1);
		uint32_t len = end_cell - start_cell;

		std::string cell = str.substr(start_cell, len);
		float val = std::stof(cell);

		vals.push_back(val);

		search_idx = end_cell;
		n_entries++;
	}

	return vals;
}

////////////////////////////////////////////////////////////////////////////////
vpHomogeneousMatrix line_to_pose(std::string line) {


	std::vector<float> vals = split_float(line, ",");

	vpHomogeneousMatrix pose(vals);

	return pose;
}

////////////////////////////////////////////////////////////////////////////////
void poses_from_file(const std::string& filepath,
	std::vector<vpHomogeneousMatrix>* poses, std::vector<int>* indices) {

	std::ifstream file(filepath);
	std::string raw_line;

	while(std::getline(file, raw_line)) {

		int i = 0;
		while (raw_line[i++] != ',') ;

		int index = std::atoi(raw_line.substr(0, i).c_str());
		indices->push_back(index);

		vpHomogeneousMatrix pose = line_to_pose(raw_line.substr(i, raw_line.length()));
		poses->push_back(pose);
	}
}

////////////////////////////////////////////////////////////////////////////////
vpHomogeneousMatrix get_pose_by_index(int index,
	std::vector<vpHomogeneousMatrix> poses, std::vector<int> indices) {

	for (uint32_t i = 0; i < poses.size(); i++) {

		if (indices[i] == index)
			return poses[indices[i]];
	}

	return vpHomogeneousMatrix();
}

////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv) {

	std::vector<vpHomogeneousMatrix> board_poses;
	std::vector<int> board_indices;
	poses_from_file("res/board_poses_corrected.csv", &board_poses, &board_indices);

	std::vector<vpHomogeneousMatrix> endeff_poses;
	std::vector<int> endeff_indices;
	poses_from_file("res/endeffector_poses.csv", &endeff_poses, &endeff_indices);
	
	std::vector<vpHomogeneousMatrix> AA;
	std::vector<vpHomogeneousMatrix> BB;

	for (uint32_t i = 0; i < board_poses.size() - 1; i++) {

		int board_idx0 = board_indices[i];
		int board_idx1 = board_indices[i + 1];
		
		vpHomogeneousMatrix A0 = board_poses[i];
		vpHomogeneousMatrix A1 = board_poses[i + 1];
		vpHomogeneousMatrix A = A1 * A0.inverse();
		AA.push_back(A);

		vpHomogeneousMatrix B0 = get_pose_by_index(board_idx0, endeff_poses, endeff_indices);
		vpHomogeneousMatrix B1 = get_pose_by_index(board_idx1, endeff_poses, endeff_indices);
		vpHomogeneousMatrix B = B1 * B0.inverse();
		BB.push_back(B);

		/*
		std::cout << "A0: " << std::endl << A0 << std::endl;
		std::cout << "A1: " << std::endl << A1 << std::endl;
		std::cout << "B0: " << std::endl << B0 << std::endl;
		std::cout << "B1: " << std::endl << B1 << std::endl;
		*/

		// std::cin.ignore();
	}

	vpHomogeneousMatrix base_to_cam;
	vpCalibration::calibrationTsai(BB, AA, base_to_cam);

	vpHomogeneousMatrix cam_to_base = base_to_cam.inverse();

	std::cout << "Base to camera: " << std::endl << base_to_cam
			  << std::endl;

	std::cout << "Camera to base: " << std::endl << cam_to_base
		   	  << std::endl;
	
	/*
	vpHomogeneousMatrix endeff_to_board_prev;

	for (uint32_t i = 0; i < board_poses.size(); i++) {

		vpHomogeneousMatrix endeff_to_board = board_poses[i].inverse()
			* base_to_cam * endeff_poses[i];
		
		if (i > 0) {

			vpHomogeneousMatrix delta;

			for (uint8_t j = 0; j < 4; j++) {

				for (uint8_t k = 0; k < 4; k++)
					delta[j][k] = std::abs(endeff_to_board[j][k]
						- endeff_to_board_prev[j][k]);
			}

			std::cout << "endeff to board delta " << i << std::endl << delta
					  << std::endl;
		}

		endeff_to_board_prev = endeff_to_board;
	}
	*/
}

/// @file

