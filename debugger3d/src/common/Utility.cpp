#include <fstream>
#include <sstream>
#include <iostream>
#include <math.h>

#include "Utility.hpp"
#include "Timer.hpp"

#define PI 3.14159265f

////////////////////////////////////////////////////////////////////////////////
bool Utility::IsBetween(Eigen::Vector3f near, Eigen::Vector3f far,
                        Eigen::Vector3f point, Eigen::Vector3f dir) {

    double dp = point.dot(dir);

    return ((dp < far.dot(dir)) && (dp > near.dot(dir)));
}

////////////////////////////////////////////////////////////////////////////////
std::vector<std::string> Utility::Split(std::string input_string,
                                        std::string delimiters) {
    
    std::vector<std::string> words;
    
    std::stringstream stringstream(input_string);
    std::string line;

    while (std::getline(stringstream, line)) {

        std::size_t prev = 0, pos;

        while ((pos = line.find_first_of(delimiters, prev)) !=
                std::string::npos) {

            if (pos > prev)
                words.push_back(line.substr(prev, pos - prev));

            prev = pos + 1;
        }
        if (prev < line.length())
            words.push_back(line.substr(prev, std::string::npos));
    }
    
    return words;
}

////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXf Utility::ReadPointcloudFromFile(const std::string& path) {

    // Allocate too much space
    Eigen::MatrixXf pointcloud(300000, 3);
    Eigen::Matrix3f rotation = Eigen::Matrix3f::Zero();
    rotation(0, 0) = 1.0f;
    rotation(1, 2) = -1.0f;
    rotation(2, 1) = 1.0f;

    std::ifstream filestream(path);

    int n_line = 0;
    
    for (std::string line; std::getline(filestream, line);) {

        std::istringstream linestream(line);

        int n_entry = 0;

        for (int i = 0; i < 3; i++) {

            std::string data;
            std::getline(linestream, data, '\t');
            pointcloud(n_line, n_entry) = std::stof(data) / 1000.0f;
            n_entry++;
        }
        /*
        Eigen::Vector3f point = pointcloud.row(n_line);
        Eigen::Vector3f rotated = rotation * point;
        pointcloud.row(n_line) = rotated;
        */
        n_line++;
    }

    // Shrink down to fit
    pointcloud.conservativeResize(n_line, Eigen::NoChange);

    return pointcloud;
}

////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix4f Utility::ReadCameraPoseFromFile(const std::string& path) {

    Eigen::Matrix4f pose;
    Eigen::Matrix4f transform = Eigen::Matrix4f::Zero();
    transform(0, 0) = 1.0f;
    transform(1, 2) = -1.0f;
    transform(2, 1) = 1.0f;
    transform(3, 3) = 1.0f;

    std::ifstream filestream(path);

    int n_row = 0;

    for (std::string line; std::getline(filestream, line);) {

        std::vector<std::string> words = Split(line, " \t");
        
        for (int i = 0; i < 4; i++) {

            pose(n_row, i) = std::stof(words[i]);
            
            if (i == 3 && n_row != 3)
                pose(n_row, i) /= 1000.0f;
        }

        n_row++;
    }

    // pose = transform * pose;
    return pose;
}

////////////////////////////////////////////////////////////////////////////////
uint32_t Utility::Index3D(uint32_t i, uint32_t j, uint32_t k, const uint32_t n_voxels_per_side) {

    return i + n_voxels_per_side * (j + n_voxels_per_side * k);
}
////////////////////////////////////////////////////////////////////////////////
uint32_t Utility::Index2D(int row, int col, int ncols) {

    return row * ncols + col;
}

////////////////////////////////////////////////////////////////////////////////
glm::vec3 Utility::InterpolateColor(float h) {

    float r, g, b;

    if (h >= 0.0f && h < 0.5f) {

        g = h / 0.5f;
        r = 1.0f - g;
        b = 0.0f;
    }
    else if (h >= 0.5f && h <= 1.0f) {

        r = 0.0f;
        b = (h - 0.4f) / 0.5f;
        g = 1.0f - b;
    }

    return glm::vec3(r, g, b);
}

////////////////////////////////////////////////////////////////////////////////
float Utility::Gaussian(float x, float mean, float var) {
    
    float prob = 1000.0f;

    if (var > 0.0e-7f) {
        prob = 1.0f / sqrt(2.0f * PI * var) * exp(-pow(x - mean, 2) / (2.0f * var));
    }

    return prob;
}

////////////////////////////////////////////////////////////////////////////////
Eigen::Vector3f Utility::ComputeCentroid(std::vector<Eigen::Vector3f>* points) {

    Eigen::Vector3f centroid(0.0f, 0.0f, 0.0f);

    for (uint32_t i = 0; i < points->size(); i++)
        centroid += (*points)[i];

    return centroid / points->size();
}

/// @file

