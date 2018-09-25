#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco/charuco.hpp>

////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv) {

	if (argc < 3) {
		
		std::cout << "Not enough input arguments." << std::endl;
		std::cout << "Usage: " << argv[0]
				  << " <n_squares_width> <n_squares_height>" << std::endl;

		exit(-1);
	}

	uint32_t n_width  = std::atoi(argv[1]);
	uint32_t n_height = std::atoi(argv[2]);

	cv::Ptr<cv::aruco::Dictionary> dictionary = 
		cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

	cv::Mat marker_image;
	cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(
		n_width, n_height, 0.04f, 0.03f, dictionary);

	board->draw(cv::Size(900, 600), marker_image);
	cv::imshow("Marker image", marker_image);
	cv::waitKey(0);

	std::string filepath = "res/board-" + std::to_string(n_width) + "x"
		+ std::to_string(n_height) + ".png";

	cv::imwrite(filepath, marker_image);
}

/// @file

