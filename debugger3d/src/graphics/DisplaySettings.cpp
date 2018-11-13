#include <string>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <vector>
#include <sstream>
#include <glm/glm.hpp>

#include "DisplaySettings.hpp"
#include "common/StringUtility.hpp"

namespace DisplaySettings {

    int WIDTH             = 1200;
    int HEIGHT            = 900;
    std::string TITLE     = "Debugger 3D";
    glm::vec3 CLEAR_COLOR = glm::vec3(0.2f, 0.2f, 0.2f);

}

/**
 * @brief Load values from a given configuration file to the
 * namespace fields
 */
void DisplaySettings::LoadSettings(const std::string& filename) {
    std::ifstream input_file(filename.c_str());
    std::string line;

    while(getline(input_file, line)) {
        std::string::iterator end_pos = std::remove(line.begin(),
                                                    line.end(), ' ');
        line.erase(end_pos, line.end());

        std::vector<std::string> tokens = StringUtility::split(line, '=');

        if(tokens[0] == "WIDTH") {
            std::stringstream value_stream(tokens[1]);
            int width;

            value_stream >> width;
            WIDTH = width;
        }
        else if(tokens[0] == "HEIGHT") {
            std::stringstream value_stream(tokens[1]);
            int height;

            value_stream >> height;
            HEIGHT = height;
        }
        else if(tokens[0] == "TITLE") {
            TITLE = tokens[1];
        }
    }
}

/// @file
