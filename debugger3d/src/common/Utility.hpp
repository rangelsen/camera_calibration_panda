#ifndef UTILITY_HPP
#define UTILITY_HPP

#include <Eigen/Eigen>
#include <string>
#include <vector>
#include <glm/glm.hpp>

namespace Utility {

    /**
     * @brief Returns true if point is between near and far along the
     * given direction
     */
    bool IsBetween(Eigen::Vector3f near, Eigen::Vector3f far,
                   Eigen::Vector3f point, Eigen::Vector3f dir);
    
    /**
     * @brief Split a string on a series of different delimiters"
     */
    std::vector<std::string> Split(std::string input_string,
                                   std::string delimiters);

    Eigen::MatrixXf ReadPointcloudFromFile(const std::string& path);

    Eigen::Matrix4f ReadCameraPoseFromFile(const std::string& path);

    uint32_t Index3D(uint32_t i, uint32_t j, uint32_t k, const uint32_t n_voxels_per_side);

    uint32_t Index2D(int i, int j, int n);

    glm::vec3 InterpolateColor(float h);

    float Gaussian(float x, float mean, float var);

    Eigen::Vector3f ComputeCentroid(std::vector<Eigen::Vector3f>* points);
}

#endif // UTILITY_HPP

/// @file

