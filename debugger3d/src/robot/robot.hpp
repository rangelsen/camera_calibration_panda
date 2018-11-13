#ifndef ROBOT_H
#define ROBOT_H

#include <vector>

////////////////////////////////////////////////////////////////////////////////
typedef enum {

	ATCH_LINK,
	ATCH_BASE,
} attatchment;

typedef enum {
	JOINT_REVOLUTE,
	JOINT_PRISMATIC,
} joint_type;

typedef struct Link {

	float length;
	attatchment atch;
} Link;

typedef struct Joint {

	joint_type type;
	float val;
} Joint;

typedef struct EndEffector {

} EndEffector;

typedef struct Robot {

	Link links[8];
	Joint joints[7];
	EndEffector endeff;
} Robot;

////////////////////////////////////////////////////////////////////////////////
std::vector<glm::mat4> robot_get_kinematic_chain(float* q, int n_joints);

#endif // ROBOT_H

/// @file

