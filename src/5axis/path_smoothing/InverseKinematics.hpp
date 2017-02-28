#ifndef INVERSE_KINEMATICS
#define INVERSE_KINEMATICS

#include "../../utils/logoutput.h"
#include <math.h>

namespace cura {

class InverseKinematics {
private:
	float pos_row1(float x_S, float x_buildplate, float y_buildplate,
		float z_buildplate, float rho, float theta, float phi, float psi, float z_offset);
	float pos_row2(float y_S, float x_buildplate, float y_buildplate,
		float z_buildplate, float rho, float phi, float psi);
	float pos_row3(float z_S, float x_buildplate, float y_buildplate,
		float z_buildplate, float rho, float theta, float phi, float psi, float z_offset);
public:
	float posMatrix[3];
	float velMatrix[3];
	float accelMatrix[3];

	InverseKinematics() {}

	float* position(float x_S, float y_S, float z_S, float x_buildplate, float y_buildplate,
		float z_buildplate, float rho, float theta, float phi, float psi, float z_offset);
};

}

#endif