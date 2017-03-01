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
		float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate, float theta_dot, float phi_dot);
	float vel_10(float forwVel_10, float forwVel_11, float forwVel_12, float forwVel_13, float forwVel_14,
		float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate, float theta_dot, float phi_dot);
	float vel_20(float forwVel_20, float forwVel_21, float forwVel_22, float forwVel_23, float forwVel_24,
		float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate, float theta_dot, float phi_dot);

	float acc_00(float forwAcc_00, float forwAcc_01, float forwAcc_02, float forwAcc_03, float forwAcc_04,
		float forwVel_00, float forwVel_01, float forwVel_02, float forwVel_03, float forwVel_04,
		float x_ddot_buildplate, float y_ddot_buildplate, float z_ddot_buildplate, float theta_ddot, float phi_ddot,
		float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate, float theta_dot, float phi_dot);

	float acc_10(float forwAcc_10, float forwAcc_11, float forwAcc_12, float forwAcc_13, float forwAcc_14,
		float forwVel_10, float forwVel_11, float forwVel_12, float forwVel_13, float forwVel_14,
		float x_ddot_buildplate, float y_ddot_buildplate, float z_ddot_buildplate, float theta_ddot, float phi_ddot,
		float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate, float theta_dot, float phi_dot);

	float acc_20(float forwAcc_20, float forwAcc_21, float forwAcc_22, float forwAcc_23, float forwAcc_24,
		float forwVel_20, float forwVel_21, float forwVel_22, float forwVel_23, float forwVel_24,
		float x_ddot_buildplate, float y_ddot_buildplate, float z_ddot_buildplate, float theta_ddot, float phi_ddot,
		float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate, float theta_dot, float phi_dot);
public:
	float posMatrix[3][1];
	float velMatrix[3][1];
	float accMatrix[3][1];

	InverseKinematics() {}

	Matrix3x1 position(float x_S, float y_S, float z_S, float x_buildplate, float y_buildplate,
		float z_buildplate, float rho, float theta, float phi, float psi, float z_offset);

	Matrix3x1 velocity(Matrix3x8 forwVelMatrix, float x_dot_buildplate, float y_dot_buildplate,
		float z_dot_buildplate, float theta_dot, float phi_dot);

	Matrix3x1 acceleration(Matrix3x8 forwAccMatrix, Matrix3x8 forwVelMatrix, float x_ddot_buildplate,
		float y_ddot_buildplate, float z_ddot_buildplate, float theta_ddot, float phi_ddot, 
		float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate, float theta_dot, float phi_dot);
};

}

#endif