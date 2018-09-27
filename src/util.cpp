#include <queue>
#include <string>
#include <sstream>
#include <iostream>

#include "util.hpp"

////////////////////////////////////////////////////////////////////////////////
#define MIN_X 0.0
#define MAX_X 0.7
#define MIN_Y -0.7
#define MAX_Y 0.7
#define MIN_Z 0.0
#define MAX_Z 1.0

////////////////////////////////////////////////////////////////////////////////
bool Util::validXYZ(double x, double y, double z) {

	bool input_x_valid = (x >= MIN_X && x <= MAX_X);
	bool input_y_valid = (y >= MIN_Y && y <= MAX_Y);
	bool input_z_valid = (z >= MIN_Z && z <= MAX_Z);
	return (input_x_valid && input_y_valid && input_z_valid);
}

////////////////////////////////////////////////////////////////////////////////
bool Util::getUserPosition(double* x, double* y, double* z) {

    std::cout << "Input: " << std::endl;

	std::string input;
	std::string command, tmp;

    std::queue<std::string> v;

    std::getline(std::cin, input);
    std::istringstream iss(input);
    std::getline(iss, command, ' ');

    if (command == "pos") {

        while (std::getline(iss, tmp, ' ')) {

            v.push(tmp);
        }

        if (v.size() != 3) {

            std::cout << "Syntax error: 3 arguments expected" << std::endl;
            return false;
        }

        *x = std::atof(v.front().c_str());
        v.pop();
        *y = std::atof(v.front().c_str());
        v.pop();
        *z = std::atof(v.front().c_str());
        v.pop();

        if (!validXYZ(*x, *y, *z)) {

            std::cout << "Setpoint out of bounds" << std::endl;
            return false;
        }

        return true;
    }
}
