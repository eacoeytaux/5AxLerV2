#ifndef INVERSE_KINEMATICS
#define INVERSE_KINEMATICS

#include "../../utils/logoutput.h"
#include "../Utility.hpp"
#include <math.h>

namespace cura {

class InverseKinematics {
private:
	float pos_00(float x_S, float x_buildplate, float y_buildplate,
		float z_buildplate, float rho, float theta, float phi, float psi, float z_offset);
	float pos_10(float y_S, float x_buildplate, float y_buildplate,
		float z_buildplate, float rho, float phi, float psi);
	float pos_20(float z_S, float x_buildplate, float y_buildplate,
		float z_buildplate, float rho, float theta, float phi, float psi, float z_offset);

	float vel_00(float forwVel_00, float forwVel_01, float forwVel_02, float forwVel_03, float forwVel_04,
		float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate);
public:
	float posMatrix[3][1];
	float velMatrix[3][1];
	float accelMatrix[3][1];

	InverseKinematics() {}

	Matrix3x1 position(float x_S, float y_S, float z_S, float x_buildplate, float y_buildplate,
		float z_buildplate, float rho, float theta, float phi, float psi, float z_offset);
	Matrix3x1 velocity();
};

}

#endif