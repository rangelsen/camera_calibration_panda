#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "robot.hpp"

static const std::vector<glm::vec3> JOINT_AXES_ {

	glm::vec3(0.0f, 0.0f, 1.0f),
	glm::vec3(0.0f, 0.0f, 1.0f),
	glm::vec3(0.0f, 0.0f, 1.0f),
	glm::vec3(0.0f, 0.0f, 1.0f),
	glm::vec3(0.0f, 0.0f, 1.0f),
	glm::vec3(0.0f, 0.0f, 1.0f),
	glm::vec3(0.0f, 0.0f, 1.0f)
};

////////////////////////////////////////////////////////////////////////////////
std::vector<glm::mat4> robot_get_kinematic_chain(float* q, int n_joints) {

	std::vector<glm::mat4> kin_chain;
	
	for (int i = 0; i < n_joints; i++) {

		glm::mat4 transform = glm::rotate(glm::mat4(1.0f), q[i], JOINT_AXES_[i]);

		kin_chain.push_back(transform);
	}

	return kin_chain;
}

/// @file

