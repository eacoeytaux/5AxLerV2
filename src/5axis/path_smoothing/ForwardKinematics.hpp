#ifndef FORWARD_KINEMATICS
#define FORWARD_KINEMATICS

#include "../../utils/logoutput.h"
#include "../Utility.hpp"

namespace cura {

class ForwardKinematics {
private:
	float pos_00(float x_frame, float x_buildplate, float y_buildplate,
		float z_buildplate, float rho, float theta, float phi, float psi, float z_offset);
	float pos_10(float y_frame, float x_buildplate, float y_buildplate,
		float z_buildplate, float rho, float phi, float psi);
	float pos_20(float z_frame, float x_buildplate, float y_buildplate,
		float z_buildplate, float rho, float theta, float phi, float psi, float z_offset);

	float vel_jacobian_00() { return 1; }
	float vel_jacobian_01() { return 0; }
	float vel_jacobian_02() { return 0; }
	float vel_jacobian_03(float x_buildplate, float y_buildplate, float z_buildplate, float rho,
		float theta, float phi, float psi, float z_offset);
	float vel_jacobian_04(float x_buildplate, float y_buildplate, float z_buildplate, float rho, float theta, float phi,
		float psi);
	float vel_jacobian_05(float rho, float theta, float phi);
	float vel_jacobian_06(float rho, float theta, float phi, float psi);
	float vel_jacobian_07(float rho, float theta, float phi, float psi);

	float vel_jacobian_10() { return 0; }
	float vel_jacobian_11() { return 1; }
	float vel_jacobian_12() { return 0; }
	float vel_jacobian_13() { return 0; }
	float vel_jacobian_14(float x_buildplate, float y_buildplate, float z_buildplate, float rho, float phi, float psi);
	float vel_jacobian_15(float rho, float phi);
	float vel_jacobian_16(float rho, float phi, float psi);
	float vel_jacobian_17(float rho, float phi, float psi);

	float vel_jacobian_20() { return 0; }
	float vel_jacobian_21() { return 0; }
	float vel_jacobian_22() { return 1; }
	float vel_jacobian_23(float x_buildplate, float y_buildplate, float z_buildplate,
		float rho, float theta, float phi, float psi, float z_offset);
	float vel_jacobian_24(float x_buildplate, float y_buildplate, float z_buildplate,
		float rho, float theta, float phi, float psi);
	float vel_jacobian_25(float rho, float theta, float phi);
	float vel_jacobian_26(float rho, float theta, float phi, float psi);
	float vel_jacobian_27(float rho, float theta, float phi, float psi);

	float acc_jacobian_00() { return 0; }
	float acc_jacobian_01() { return 0; }
	float acc_jacobian_02() { return 0; }
	float acc_jacobian_03(float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate, float theta_dot,
		float phi_dot,float x_buildplate, float y_buildplate, float z_buildplate, float rho, float theta,
		float phi, float psi, float z_offset);
	float acc_jacobian_04(float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate, float theta_dot,
		float phi_dot,float x_buildplate, float y_buildplate, float z_buildplate, float rho, float theta,
		float phi, float psi);
	float acc_jacobian_05(float theta_dot, float phi_dot, float rho, float theta, float phi);
	float acc_jacobian_06(float theta_dot, float phi_dot, float rho, float theta, float phi, float psi);
	float acc_jacobian_07(float theta_dot, float phi_dot, float rho, float theta, float phi, float psi);

	float acc_jacobian_10() { return 0; }
	float acc_jacobian_11() { return 0; }
	float acc_jacobian_12() { return 0; }
	float acc_jacobian_13() { return 0; }
	float acc_jacobian_14(float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate, float phi_dot,
		float x_buildplate, float y_buildplate, float z_buildplate, float rho, float phi, float psi);
	float acc_jacobian_15(float phi_dot, float rho, float phi);
	float acc_jacobian_16(float phi_dot, float rho, float phi, float psi);
	float acc_jacobian_17(float phi_dot, float rho, float phi, float psi);

	float acc_jacobian_20() { return 0; }
	float acc_jacobian_21() { return 0; }
	float acc_jacobian_22() { return 0; }
	float acc_jacobian_23(float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate, float theta_dot,
		float phi_dot,float x_buildplate, float y_buildplate, float z_buildplate, float rho, float theta,
		float phi, float psi);
	float acc_jacobian_24(float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate, float theta_dot,
		float phi_dot,float x_buildplate, float y_buildplate, float z_buildplate, float rho, float theta,
		float phi, float psi);
	float acc_jacobian_25(float theta_dot, float phi_dot, float rho, float theta, float phi);
	float acc_jacobian_26(float theta_dot, float phi_dot, float rho, float theta, float phi, float psi);
	float acc_jacobian_27(float theta_dot, float phi_dot, float rho, float theta, float phi, float psi);

public:
	// Matrix dimensions are [# rows][# cols]
	float posMatrix[3][1];
	float velMatrix[3][8];
	float accMatrix[3][8];

	ForwardKinematics() {}

	Matrix3x1 position(float x_frame, float y_frame, float z_frame, float x_buildplate, 
		float y_buildplate, float z_buildplate, float rho, float theta, float phi,
		float psi, float z_offset);

	Matrix3x8 velocity_jacobian(float x_buildplate, float y_buildplate, float z_buildplate,
		float rho, float theta, float phi, float psi, float z_offset);

	Matrix3x8 acceleration_jacobian(float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate,
		float theta_dot, float phi_dot, float x_buildplate, float y_buildplate, float z_buildplate,
		float rho, float theta, float phi, float psi, float z_offset);
};

}

#endif