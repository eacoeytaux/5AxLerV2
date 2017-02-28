#ifndef FORWARD_KINEMATICS
#define FORWARD_KINEMATICS

#include "../../utils/logoutput.h"

namespace cura {

class ForwardKinematics {
private:
	float pos_row1(float x_frame, float x_buildplate, float y_buildplate,
		float z_buildplate, float rho, float theta, float phi, float psi, float z_offset);
	float pos_row2(float y_frame, float x_buildplate, float y_buildplate,
		float z_buildplate, float rho, float phi, float psi);
	float pos_row3(float z_frame, float x_buildplate, float y_buildplate,
		float z_buildplate, float rho, float theta, float phi, float psi, float z_offset);

public:
	// Matrix dimensions are [# rows][# cols]
	float posMatrix[3];
	float velMatrix[3][8];
	float accMatrix[3][8];

	ForwardKinematics() {}

	float* position(float x_frame, float y_frame, float z_frame, float x_buildplate, 
		float y_buildplate, float z_buildplate, float rho, float theta, float phi,
		float psi, float z_offset);

	float** velocity();
	float** acceleration();
};

}

#endif