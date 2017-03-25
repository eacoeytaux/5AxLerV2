#ifndef INVERSE_KINEMATICS
#define INVERSE_KINEMATICS

#include "../../utils/logoutput.h"
#include "../Utility.hpp"
#include <math.h>

namespace cura {

class InverseKinematics {
private:
	float pos_frame_00(float S_00, float x_buildplate, float y_buildplate, float z_buildplate, float s_rho,
		float c_rho, float s_theta, float c_theta, float s_phi, float c_phi, float s_psi, float c_psi,
		float z_offset);
	float pos_frame_10(float S_10, float x_buildplate, float y_buildplate, float z_buildplate, float s_rho,
		float c_rho, float s_phi, float c_phi, float s_psi, float c_psi);
	float pos_frame_20(float S_20, float x_buildplate, float y_buildplate, float z_buildplate, float s_rho,
		float c_rho, float s_theta, float c_theta, float s_phi, float c_phi, float s_psi, float c_psi,
		float z_offset);

	float vel_frame_00(float jacobian_03, float jacobian_04, float jacobian_05, float jacobian_06, float forwVel_07,
		float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate, float theta_dot, float phi_dot);
	float vel_frame_10(float jacobian_13, float jacobian_14, float jacobian_15, float jacobian_16, float jacobian_17,
		float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate, float theta_dot, float phi_dot);
	float vel_frame_20(float jacobian_23, float jacobian_24, float jacobian_25, float jacobian_26, float jacobian_27,
		float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate, float theta_dot, float phi_dot);

	float acc_frame_00(float forwAcc_03, float forwAcc_04, float forwAcc_05, float forwAcc_06, float forwAcc_07,
		float forwVel_03, float forwVel_04, float forwVel_05, float forwVel_06, float forwVel_07,
		float x_ddot_buildplate, float y_ddot_buildplate, float z_ddot_buildplate, float theta_ddot, float phi_ddot,
		float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate, float theta_dot, float phi_dot);

	float acc_frame_10(float forwAcc_13, float forwAcc_14, float forwAcc_15, float forwAcc_16, float forwAcc_17,
		float forwVel_13, float forwVel_14, float forwVel_15, float forwVel_16, float forwVel_17,
		float x_ddot_buildplate, float y_ddot_buildplate, float z_ddot_buildplate, float theta_ddot, float phi_ddot,
		float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate, float theta_dot, float phi_dot);

	float acc_frame_20(float forwAcc_23, float forwAcc_24, float forwAcc_25, float forwAcc_26, float forwAcc_27,
		float forwVel_23, float forwVel_24, float forwVel_25, float forwVel_26, float forwVel_27,
		float x_ddot_buildplate, float y_ddot_buildplate, float z_ddot_buildplate, float theta_ddot, float phi_ddot,
		float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate, float theta_dot, float phi_dot);

	float pos_buildplate_00(Matrix3x1 constMatrix, float s_rho, float c_rho, float s_theta,	float c_theta,
		float s_phi, float c_phi);
	float pos_buildplate_10(Matrix3x1 constMatrix, float s_rho, float c_rho, float s_theta,	float c_theta,
		float s_phi, float c_phi, float s_psi, float c_psi);
	float pos_buildplate_20(Matrix3x1 constMatrix, float s_rho, float c_rho, float s_theta,	float c_theta,
		float s_phi, float c_phi, float s_psi, float c_psi);

	float vel_buildplate_00(Matrix3x1 constMatrix, float ij_00, float ij_01, float ij_02);
	float vel_buildplate_10(Matrix3x1 constMatrix, float ij_10, float ij_11, float ij_12);
	float vel_buildplate_20(Matrix3x1 constMatrix, float ij_20, float ij_21, float ij_22);

	float acc_buildplate_00(Matrix3x1 constMatrix, float ij_00, float ij_01, float ij_02);
	float acc_buildplate_10(Matrix3x1 constMatrix, float ij_10, float ij_11, float ij_12);
	float acc_buildplate_20(Matrix3x1 constMatrix, float ij_20, float ij_21, float ij_22);
public:
	float pos_frameMatrix[3][1];
	float vel_frameMatrix[3][1];
	float acc_frameMatrix[3][1];
	float pos_buildplateMatrix[3][1];
	float vel_buildplateMatrix[3][1];
	float acc_buildplateMatrix[3][1];

	InverseKinematics() {}

	Matrix3x1 position_frame(Matrix3x1 S, float x_buildplate, float y_buildplate,
		float z_buildplate, float rho, float theta, float phi, float psi, float z_offset);

	Matrix3x1 velocity_frame(Matrix3x8 velJacobian, float x_dot_buildplate, float y_dot_buildplate,
		float z_dot_buildplate, float theta_dot, float phi_dot);

	Matrix3x1 acceleration_frame(Matrix3x8 accJacobian, Matrix3x8 velJacobian, float x_ddot_buildplate,
		float y_ddot_buildplate, float z_ddot_buildplate, float theta_ddot, float phi_ddot, float x_dot_buildplate,
		float y_dot_buildplate,	float z_dot_buildplate, float theta_dot, float phi_dot);

	Matrix3x1 position_buildplate(Matrix3x1 S, Matrix3x1 invPosFrameMatrix, float rho, float theta, float phi,
		float psi, float z_offset);

	Matrix3x1 velocity_buildplate(Matrix3x8 velJacobian, Matrix3x1 invVelFrameMatrix, float theta_dot,
		float phi_dot);

	Matrix3x1 acceleration_buildplate(Matrix3x8 velJacobian, Matrix3x8 accJacobian, Matrix3x1 invAccFrameMatrix,
		float theta_ddot, float phi_ddot, float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate,
		float theta_dot, float phi_dot);
};

}

#endif