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

	float vel_00() { return 1; }
	float vel_01() { return 0; }
	float vel_02() { return 0; }
	float vel_03(float x_buildplate, float y_buildplate, float z_buildplate, float rho,
		float theta, float phi, float psi, float z_offset);
	float vel_04(float x_buildplate, float y_buildplate, float z_buildplate, float rho, float theta, float phi,
		float psi);
	float vel_05(float rho, float theta, float phi);
	float vel_06(float rho, float theta, float phi, float psi);
	float vel_07(float rho, float theta, float phi, float psi);

	float vel_10() { return 0; }
	float vel_11() { return 1; }
	float vel_12() { return 0; }
	float vel_13() { return 0; }
	float vel_14(float x_buildplate, float y_buildplate, float z_buildplate, float rho, float phi, float psi);
	float vel_15(float rho, float phi);
	float vel_16(float rho, float phi, float psi);
	float vel_17(float rho, float phi, float psi);

	float vel_20() { return 0; }
	float vel_21() { return 0; }
	float vel_22() { return 1; }
	float vel_23(float x_buildplate, float y_buildplate, float z_buildplate,
		float rho, float theta, float phi, float psi, float z_offset);
	float vel_24(float x_buildplate, float y_buildplate, float z_buildplate,
		float rho, float theta, float phi, float psi);
	float vel_25(float rho, float theta, float phi);
	float vel_26(float rho, float theta, float phi, float psi);
	float vel_27(float rho, float theta, float phi, float psi);

	float acc_00() { return 0; }
	float acc_01() { return 0; }
	float acc_02() { return 0; }
	float acc_03(float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate, float theta_dot,
		float phi_dot,float x_buildplate, float y_buildplate, float z_buildplate, float rho, float theta,
		float phi, float psi, float z_offset);
	float acc_04(float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate, float theta_dot,
		float phi_dot,float x_buildplate, float y_buildplate, float z_buildplate, float rho, float theta,
		float phi, float psi);
	float acc_05(float theta_dot, float phi_dot, float rho, float theta, float phi);
	float acc_06(float theta_dot, float phi_dot, float rho, float theta, float phi, float psi);
	float acc_07(float theta_dot, float phi_dot, float rho, float theta, float phi, float psi);

	float acc_10() { return 0; }
	float acc_11() { return 0; }
	float acc_12() { return 0; }
	float acc_13() { return 0; }
	float acc_14(float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate, float phi_dot,
		float x_buildplate, float y_buildplate, float z_buildplate, float rho, float phi, float psi);
	float acc_15(float phi_dot, float rho, float phi);
	float acc_16(float phi_dot, float rho, float phi, float psi);
	float acc_17(float phi_dot, float rho, float phi, float psi);

	float acc_20() { return 0; }
	float acc_21() { return 0; }
	float acc_22() { return 0; }
	float acc_23(float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate, float theta_dot,
		float phi_dot,float x_buildplate, float y_buildplate, float z_buildplate, float rho, float theta,
		float phi, float psi);
	float acc_24(float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate, float theta_dot,
		float phi_dot,float x_buildplate, float y_buildplate, float z_buildplate, float rho, float theta,
		float phi, float psi);
	float acc_25(float theta_dot, float phi_dot, float rho, float theta, float phi);
	float acc_26(float theta_dot, float phi_dot, float rho, float theta, float phi, float psi);
	float acc_27(float theta_dot, float phi_dot, float rho, float theta, float phi, float psi);

public:
	// Matrix dimensions are [# rows][# cols]
	float posMatrix[3][1];
	float velMatrix[3][8];
	float accMatrix[3][8];

	ForwardKinematics() {}

	Matrix3x1 position(float x_frame, float y_frame, float z_frame, float x_buildplate, 
		float y_buildplate, float z_buildplate, float rho, float theta, float phi,
		float psi, float z_offset);

	Matrix3x8 velocity(float x_buildplate, float y_buildplate, float z_buildplate,
		float rho, float theta, float phi, float psi, float z_offset);

	Matrix3x8 acceleration(float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate,
	float theta_dot, float phi_dot, float x_buildplate, float y_buildplate, float z_buildplate,
	float rho, float theta, float phi, float psi, float z_offset);
};

}

#endif