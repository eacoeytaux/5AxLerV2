#include "InverseKinematics.hpp"

namespace cura {

float InverseKinematics::pos_frame_00(float S_00, float x_buildplate, float y_buildplate, float z_buildplate,
	float s_rho, float c_rho, float s_theta, float c_theta, float s_phi, float c_phi, float s_psi, float c_psi,
	float z_offset) {
	float commonExpr = (c_rho * s_theta + c_phi * c_theta * s_rho);

	return		S_00
			-	(	
					  x_buildplate * (s_rho * s_theta - c_phi * c_rho * c_theta)
					- z_offset * s_theta
					+ y_buildplate * (s_psi * commonExpr - c_psi * c_theta * s_phi)
					+ z_buildplate * (c_psi * commonExpr + c_theta * s_phi * s_psi)
				);
}

float InverseKinematics::pos_frame_10(float S_10, float x_buildplate, float y_buildplate, float z_buildplate,
	float s_rho, float c_rho, float s_phi, float c_phi, float s_psi, float c_psi) {
	return		S_10
			-	(
					  y_buildplate * (c_phi * c_psi + s_phi * s_psi * s_rho)
					- z_buildplate * (c_phi * s_psi - c_psi * s_phi * s_rho)
					+ x_buildplate * c_rho * s_phi
				);
}

float InverseKinematics::pos_frame_20(float S_20, float x_buildplate, float y_buildplate, float z_buildplate,
	float s_rho, float c_rho, float s_theta, float c_theta, float s_phi, float c_phi, float s_psi, float c_psi,
	float z_offset) {
	return		S_20
			-	(
					  z_offset
					- x_buildplate * (c_theta * s_rho - c_phi * c_rho * s_theta)
					- z_offset * c_theta
					+ y_buildplate * (s_psi * (c_rho * c_theta - c_phi * s_rho * s_theta) + c_psi * s_phi * s_theta)
					+ z_buildplate * (c_psi * c_rho * c_theta - c_phi * s_rho * s_theta)
					- s_phi * s_psi * s_theta
				);
}

float InverseKinematics::vel_frame_00(float jacobian_03, float jacobian_04, float jacobian_05, float jacobian_06, float jacobian_07,
	float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate, float theta_dot, float phi_dot) {
	return	-	(
					  jacobian_03 * x_dot_buildplate
					+ jacobian_04 * y_dot_buildplate
					+ jacobian_05 * z_dot_buildplate
					+ jacobian_06 * theta_dot
					+ jacobian_07 * phi_dot
				);
}

float InverseKinematics::vel_frame_10(float jacobian_13, float jacobian_14, float jacobian_15, float jacobian_16, float jacobian_17,
	float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate, float theta_dot, float phi_dot) {
	return	-	(
					  jacobian_13 * x_dot_buildplate
					+ jacobian_14 * y_dot_buildplate
					+ jacobian_15 * z_dot_buildplate
					+ jacobian_16 * theta_dot
					+ jacobian_17 * phi_dot
				);
}

float InverseKinematics::vel_frame_20(float jacobian_23, float jacobian_24, float jacobian_25, float jacobian_26, float jacobian_27,
	float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate, float theta_dot, float phi_dot) {
	return	-	(
					  jacobian_23 * x_dot_buildplate
					+ jacobian_24 * y_dot_buildplate
					+ jacobian_25 * z_dot_buildplate
					+ jacobian_26 * theta_dot
					+ jacobian_27 * phi_dot
				);
}

float InverseKinematics::acc_frame_00(float accJacobian_03, float accJacobian_04, float accJacobian_05, float accJacobian_06, float accJacobian_07,
	float velJacobian_03, float velJacobian_04, float velJacobian_05, float velJacobian_06, float velJacobian_07,
	float x_ddot_buildplate, float y_ddot_buildplate, float z_ddot_buildplate, float theta_ddot, float phi_ddot,
	float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate, float theta_dot, float phi_dot) {
	float velComponent =		accJacobian_03 * x_dot_buildplate
							+	accJacobian_04 * y_dot_buildplate
							+	accJacobian_05 * z_dot_buildplate
							+	accJacobian_06 * theta_dot
							+	accJacobian_07 * phi_dot;

	float accComponent =		velJacobian_03 * x_ddot_buildplate
							+	velJacobian_04 * y_ddot_buildplate
							+	velJacobian_05 * z_ddot_buildplate
							+	velJacobian_06 * theta_ddot
							+	velJacobian_07 * phi_ddot;

	return -velComponent - accComponent;
}

float InverseKinematics::acc_frame_10(float accJacobian_13, float accJacobian_14, float accJacobian_15, float accJacobian_16, float accJacobian_17,
	float velJacobian_13, float velJacobian_14, float velJacobian_15, float velJacobian_16, float velJacobian_17,
	float x_ddot_buildplate, float y_ddot_buildplate, float z_ddot_buildplate, float theta_ddot, float phi_ddot,
	float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate, float theta_dot, float phi_dot) {
	float velComponent =		accJacobian_13 * x_dot_buildplate
							+	accJacobian_14 * y_dot_buildplate
							+	accJacobian_15 * z_dot_buildplate
							+	accJacobian_16 * theta_dot
							+	accJacobian_17 * phi_dot;

	float accComponent =		velJacobian_13 * x_ddot_buildplate
							+	velJacobian_14 * y_ddot_buildplate
							+	velJacobian_15 * z_ddot_buildplate
							+	velJacobian_16 * theta_ddot
							+	velJacobian_17 * phi_ddot;

	return -velComponent - accComponent;
}

float InverseKinematics::acc_frame_20(float accJacobian_23, float accJacobian_24, float accJacobian_25, float accJacobian_26, float accJacobian_27,
	float velJacobian_23, float velJacobian_24, float velJacobian_25, float velJacobian_26, float velJacobian_27,
	float x_ddot_buildplate, float y_ddot_buildplate, float z_ddot_buildplate, float theta_ddot, float phi_ddot,
	float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate, float theta_dot, float phi_dot) {
	float velComponent =		accJacobian_23 * x_dot_buildplate
							+	accJacobian_24 * y_dot_buildplate
							+	accJacobian_25 * z_dot_buildplate
							+	accJacobian_26 * theta_dot
							+	accJacobian_27 * phi_dot;

	float accComponent =		velJacobian_23 * x_ddot_buildplate
							+	velJacobian_24 * y_ddot_buildplate
							+	velJacobian_25 * z_ddot_buildplate
							+	velJacobian_26 * theta_ddot
							+	velJacobian_27 * phi_ddot;

	return -velComponent - accComponent;
}

float InverseKinematics::pos_buildplate_00(Matrix3x1 constMatrix, float s_rho, float c_rho, float s_theta,	float c_theta,
	float s_phi, float c_phi) {
	float other_00 = c_phi * c_rho * c_theta - s_rho * s_theta;
	float other_01 = c_rho * s_phi;
	float other_02 = -c_theta * s_rho - c_phi * c_rho * s_theta;

	return		other_00 * constMatrix[0][0]
			+	other_01 * constMatrix[1][0]
			+	other_02 * constMatrix[2][0];
}

float InverseKinematics::pos_buildplate_10(Matrix3x1 constMatrix, float s_rho, float c_rho, float s_theta,	float c_theta,
	float s_phi, float c_phi, float s_psi, float c_psi) {
	float other_00 = c_rho * s_psi * s_theta - c_psi * c_theta * s_phi + c_phi * c_theta * s_psi * s_rho;
	float other_01 = c_phi * c_psi + s_phi * s_psi * s_rho;
	float other_02 = c_rho * c_theta * s_psi + c_psi * s_phi * s_theta - c_phi * s_psi * s_rho * s_theta;

	return		other_00 * constMatrix[0][0]
			+	other_01 * constMatrix[1][0]
			+	other_02 * constMatrix[2][0];
}

float InverseKinematics::pos_buildplate_20(Matrix3x1 constMatrix, float s_rho, float c_rho, float s_theta,	float c_theta,
	float s_phi, float c_phi, float s_psi, float c_psi) {
	float other_00 = c_psi * c_rho * s_theta + c_theta * s_phi * s_psi + c_phi * c_psi * c_theta * s_rho;
	float other_01 = c_psi * s_phi * s_rho - c_phi * s_psi;
	float other_02 = c_psi * c_rho * c_theta - s_phi * s_psi * s_theta - c_phi * c_psi * s_rho * s_theta;

	return		other_00 * constMatrix[0][0]
			+	other_01 * constMatrix[1][0]
			+	other_02 * constMatrix[2][0];
}

float InverseKinematics::vel_buildplate_00(Matrix3x1 constMatrix, float ij_00, float ij_01, float ij_02) {
	return	-
			(	
					ij_00 * constMatrix[0][0]
				+	ij_01 * constMatrix[1][0]
				+	ij_02 * constMatrix[2][0]
			);
}

float InverseKinematics::vel_buildplate_10(Matrix3x1 constMatrix, float ij_10, float ij_11, float ij_12) {
	return	-
			(
					ij_10 * constMatrix[0][0]
				+	ij_11 * constMatrix[1][0]
				+	ij_12 * constMatrix[2][0]
			);
}

float InverseKinematics::vel_buildplate_20(Matrix3x1 constMatrix, float ij_20, float ij_21, float ij_22) {
	return	-
			(
					ij_20 * constMatrix[0][0]
				+	ij_21 * constMatrix[1][0]
				+	ij_22 * constMatrix[2][0]
			);
}

float InverseKinematics::acc_buildplate_00(Matrix3x1 constMatrix, float ij_00, float ij_01, float ij_02) {
	return	-
			(	
					ij_00 * constMatrix[0][0]
				+	ij_01 * constMatrix[1][0]
				+	ij_02 * constMatrix[2][0]
			);
}

float InverseKinematics::acc_buildplate_10(Matrix3x1 constMatrix, float ij_10, float ij_11, float ij_12) {
	return	-
			(
					ij_10 * constMatrix[0][0]
				+	ij_11 * constMatrix[1][0]
				+	ij_12 * constMatrix[2][0]
			);
}

float InverseKinematics::acc_buildplate_20(Matrix3x1 constMatrix, float ij_20, float ij_21, float ij_22) {
	return	-
			(
					ij_20 * constMatrix[0][0]
				+	ij_21 * constMatrix[1][0]
				+	ij_22 * constMatrix[2][0]
			);
}

Matrix3x1 InverseKinematics::position_frame(Matrix3x1 S, float x_buildplate, float y_buildplate,
	float z_buildplate, float rho, float theta, float phi, float psi, float z_offset) {
	float r1, r2, r3;
	float s_rho, s_theta, s_phi, s_psi, c_rho, c_theta, c_phi, c_psi;
	s_rho = sin(rho);
	s_theta = sin(theta);
	s_phi = sin(phi);
	s_psi = sin(psi);
	c_rho = cos(rho);
	c_theta = cos(theta);
	c_phi = cos(phi);
	c_psi = cos(psi);

	r1 = pos_frame_00(S[0][0], x_buildplate, y_buildplate, z_buildplate, s_rho, c_rho, s_theta, c_theta, s_phi,
		c_phi, s_psi, c_psi, z_offset);
	r2 = pos_frame_10(S[1][0], x_buildplate, y_buildplate, z_buildplate, s_rho, c_rho, s_phi, c_phi, s_psi,
		c_psi);
	r3 = pos_frame_20(S[2][0], x_buildplate, y_buildplate, z_buildplate, s_rho, c_rho, s_theta, c_theta, s_phi,
		c_phi, s_psi, c_psi, z_offset);

	pos_frameMatrix[0][0] = r1;
	pos_frameMatrix[1][0] = r2;
	pos_frameMatrix[2][0] = r3;

	return pos_frameMatrix;
}

Matrix3x1 InverseKinematics::velocity_frame(Matrix3x8 velJacobian, float x_dot_buildplate, float y_dot_buildplate,
	float z_dot_buildplate, float theta_dot, float phi_dot) {
	float r1, r2, r3;
	r1 = vel_frame_00(velJacobian[0][3], velJacobian[0][4], velJacobian[0][5], velJacobian[0][6],
		velJacobian[0][7], x_dot_buildplate, y_dot_buildplate, z_dot_buildplate, theta_dot, phi_dot);
	r2 = vel_frame_10(velJacobian[1][3], velJacobian[1][4], velJacobian[1][5], velJacobian[1][6],
		velJacobian[1][7], x_dot_buildplate, y_dot_buildplate, z_dot_buildplate, theta_dot, phi_dot);
	r3 = vel_frame_20(velJacobian[2][3], velJacobian[2][4], velJacobian[2][5], velJacobian[2][6],
		velJacobian[2][7], x_dot_buildplate, y_dot_buildplate, z_dot_buildplate, theta_dot, phi_dot);
	
	vel_frameMatrix[0][0] = r1;
	vel_frameMatrix[1][0] = r2;
	vel_frameMatrix[2][0] = r3;

	return vel_frameMatrix;
}

Matrix3x1 InverseKinematics::acceleration_frame(Matrix3x8 accJacobian, Matrix3x8 velJacobian,
	float x_ddot_buildplate, float y_ddot_buildplate, float z_ddot_buildplate, float theta_ddot, float phi_ddot,
	float x_dot_buildplate, float y_dot_buildplate,	float z_dot_buildplate, float theta_dot, float phi_dot) {
	float r1, r2, r3;
	r1 = acc_frame_00(accJacobian[0][3], accJacobian[0][4], accJacobian[0][5], accJacobian[0][6],
		accJacobian[0][7], velJacobian[0][3], velJacobian[0][4], velJacobian[0][5], velJacobian[0][6],
		velJacobian[0][7], x_ddot_buildplate, y_ddot_buildplate, z_ddot_buildplate, theta_ddot, phi_ddot,
		x_dot_buildplate, y_dot_buildplate, z_dot_buildplate, theta_dot, phi_dot);
	r2 = acc_frame_10(accJacobian[1][3], accJacobian[1][4], accJacobian[1][5], accJacobian[1][6],
		accJacobian[1][7], velJacobian[1][3], velJacobian[1][4], velJacobian[1][5], velJacobian[1][6],
		velJacobian[1][7], x_ddot_buildplate, y_ddot_buildplate, z_ddot_buildplate, theta_ddot, phi_ddot,
		x_dot_buildplate, y_dot_buildplate, z_dot_buildplate, theta_dot, phi_dot);
	r3 = acc_frame_20(accJacobian[2][3], accJacobian[2][4], accJacobian[2][5], accJacobian[2][6],
		accJacobian[2][7], velJacobian[2][3], velJacobian[2][4], velJacobian[2][5], velJacobian[2][6],
		velJacobian[2][7], x_ddot_buildplate, y_ddot_buildplate, z_ddot_buildplate, theta_ddot, phi_ddot,
		x_dot_buildplate, y_dot_buildplate, z_dot_buildplate, theta_dot, phi_dot);
	
	acc_frameMatrix[0][0] = r1;
	acc_frameMatrix[1][0] = r2;
	acc_frameMatrix[2][0] = r3;

	return acc_frameMatrix;
}

Matrix3x1 InverseKinematics::position_buildplate(Matrix3x1 S, Matrix3x1 invPosFrameMatrix, float rho,
	float theta, float phi, float psi, float z_offset) {
	float s_rho, s_theta, s_phi, s_psi, c_rho, c_theta, c_phi, c_psi;
	float constMatrix[3][1];
	float r1, r2, r3;

	s_rho = sin(rho);
	s_theta = sin(theta);
	s_phi = sin(phi);
	s_psi = sin(psi);
	c_rho = cos(rho);
	c_theta = cos(theta);
	c_phi = cos(phi);
	c_psi = cos(psi);

	constMatrix[0][0] = S[0][0] - invPosFrameMatrix[0][0] + z_offset * s_theta;
	constMatrix[1][0] = S[1][0] - invPosFrameMatrix[1][0];
	constMatrix[2][0] = S[2][0] - invPosFrameMatrix[2][0] - z_offset * (1 - c_theta);

	r1 = pos_buildplate_00(constMatrix, s_rho, c_rho, s_theta, c_theta, s_phi, c_phi);
	r2 = pos_buildplate_10(constMatrix, s_rho, c_rho, s_theta, c_theta, s_phi, c_phi, s_psi, c_psi);
	r3 = pos_buildplate_20(constMatrix, s_rho, c_rho, s_theta, c_theta, s_phi, c_phi, s_psi, c_psi);

	pos_buildplateMatrix[0][0] = r1;
	pos_buildplateMatrix[1][0] = r2;
	pos_buildplateMatrix[2][0] = r3;

	return pos_buildplateMatrix;
}

Matrix3x1 InverseKinematics::velocity_buildplate(Matrix3x8 velJacobian, Matrix3x1 invVelFrameMatrix, float theta_dot,
	float phi_dot) {
	float j1, j2, j3, j4, j5, j6, j7, j8, j9, j10, j11, j12, j13, j14, j15;
	float inverseJacobian[3][3];
	float constMatrix[3][1];
	float r1, r2, r3;

	j1 = velJacobian[0][3];
	j2 = velJacobian[0][4];
	j3 = velJacobian[0][5];
	j4 = velJacobian[0][6];
	j5 = velJacobian[0][7];
	j6 = velJacobian[1][3];
	j7 = velJacobian[1][4];
	j8 = velJacobian[1][5];
	j9 = velJacobian[1][6];
	j10 = velJacobian[1][7];
	j11 = velJacobian[2][3];
	j12 = velJacobian[2][4];
	j13 = velJacobian[2][5];
	j14 = velJacobian[2][6];
	j15 = velJacobian[2][7];

	inverseJacobian[0][0] = (j7 * j13 - j8 * j12) / (j1 * j7 * j13 - j1 * j8 * j12 - j2 * j6 * j13 + j2 * j8 * j11 + j3 * j6 * j12 - j3 * j7 * j11);
	inverseJacobian[0][1] = -(j2 * j13 - j3 * j12) / (j1 * j7 * j13 - j1 * j8 * j12 - j2 * j6 * j13 + j2 * j8 * j11 + j3 * j6 * j12 - j3 * j7 * j11);
	inverseJacobian[0][2] = (j2 * j8 - j3 * j7) / (j1 * j7 * j13 - j1 * j8 * j12 - j2 * j6 * j13 + j2 * j8 * j11 + j3 * j6 * j12 - j3 * j7 * j11);
	inverseJacobian[1][0] = -(j6 * j13 - j8 * j11) / (j1 * j7 * j13 - j1 * j8 * j12 - j2 * j6 * j13 + j2 * j8 * j11 + j3 * j6 * j12 - j3 * j7 * j11);
	inverseJacobian[1][1] = (j1 * j13 - j3 * j11) / (j1 * j7 * j13 - j1 * j8 * j12 - j2 * j6 * j13 + j2 * j8 * j11 + j3 * j6 * j12 - j3 * j7 * j11);
	inverseJacobian[1][2] = -(j1 * j8 - j3 * j6) / (j1 * j7 * j13 - j1 * j8 * j12 - j2 * j6 * j13 + j2 * j8 * j11 + j3 * j6 * j12 - j3 * j7 * j11);
	inverseJacobian[2][0] = (j6 * j12 - j7 * j11) / (j1 * j7 * j13 - j1 * j8 * j12 - j2 * j6 * j13 + j2 * j8 * j11 + j3 * j6 * j12 - j3 * j7 * j11);
	inverseJacobian[2][1] = -(j1 * j12 - j2 * j11) / (j1 * j7 * j13 - j1 * j8 * j12 - j2 * j6 * j13 + j2 * j8 * j11 + j3 * j6 * j12 - j3 * j7 * j11);
	inverseJacobian[2][2] = (j1 * j7 - j2 * j6) / (j1 * j7 * j13 - j1 * j8 * j12 - j2 * j6 * j13 + j2 * j8 * j11 + j3 * j6 * j12 - j3 * j7 * j11);

	constMatrix[0][0] = invVelFrameMatrix[0][0] + (j4 * theta_dot + j5 * phi_dot);
	constMatrix[1][0] = invVelFrameMatrix[1][0] + (j9 * theta_dot + j10 * phi_dot);
	constMatrix[2][0] = invVelFrameMatrix[2][0] + (j14 * theta_dot + j15 * phi_dot);

	r1 = vel_buildplate_00(constMatrix, inverseJacobian[0][0], inverseJacobian[0][1], inverseJacobian[0][2]);
	r2 = vel_buildplate_10(constMatrix, inverseJacobian[1][0], inverseJacobian[1][1], inverseJacobian[1][2]);
	r3 = vel_buildplate_20(constMatrix, inverseJacobian[2][0], inverseJacobian[2][1], inverseJacobian[2][2]);

	vel_buildplateMatrix[0][0] = r1;
	vel_buildplateMatrix[1][0] = r2;
	vel_buildplateMatrix[2][0] = r3;

	return vel_buildplateMatrix;
}

Matrix3x1 InverseKinematics::acceleration_buildplate(Matrix3x8 velJacobian, Matrix3x8 accJacobian, Matrix3x1 invAccFrameMatrix,
	float theta_ddot, float phi_ddot, float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate,
	float theta_dot, float phi_dot) {
	float j1, j2, j3, j4, j5, j6, j7, j8, j9, j10, j11, j12, j13, j14, j15;
	float inverseJacobian[3][3];
	float constMatrix[3][1];
	float r1, r2, r3;

	j1 = velJacobian[0][3];
	j2 = velJacobian[0][4];
	j3 = velJacobian[0][5];
	j4 = velJacobian[0][6];
	j5 = velJacobian[0][7];
	j6 = velJacobian[1][3];
	j7 = velJacobian[1][4];
	j8 = velJacobian[1][5];
	j9 = velJacobian[1][6];
	j10 = velJacobian[1][7];
	j11 = velJacobian[2][3];
	j12 = velJacobian[2][4];
	j13 = velJacobian[2][5];
	j14 = velJacobian[2][6];
	j15 = velJacobian[2][7];

	inverseJacobian[0][0] = (j7 * j13 - j8 * j12) / (j1 * j7 * j13 - j1 * j8 * j12 - j2 * j6 * j13 + j2 * j8 * j11 + j3 * j6 * j12 - j3 * j7 * j11);
	inverseJacobian[0][1] = -(j2 * j13 - j3 * j12) / (j1 * j7 * j13 - j1 * j8 * j12 - j2 * j6 * j13 + j2 * j8 * j11 + j3 * j6 * j12 - j3 * j7 * j11);
	inverseJacobian[0][2] = (j2 * j8 - j3 * j7) / (j1 * j7 * j13 - j1 * j8 * j12 - j2 * j6 * j13 + j2 * j8 * j11 + j3 * j6 * j12 - j3 * j7 * j11);
	inverseJacobian[1][0] = -(j6 * j13 - j8 * j11) / (j1 * j7 * j13 - j1 * j8 * j12 - j2 * j6 * j13 + j2 * j8 * j11 + j3 * j6 * j12 - j3 * j7 * j11);
	inverseJacobian[1][1] = (j1 * j13 - j3 * j11) / (j1 * j7 * j13 - j1 * j8 * j12 - j2 * j6 * j13 + j2 * j8 * j11 + j3 * j6 * j12 - j3 * j7 * j11);
	inverseJacobian[1][2] = -(j1 * j8 - j3 * j6) / (j1 * j7 * j13 - j1 * j8 * j12 - j2 * j6 * j13 + j2 * j8 * j11 + j3 * j6 * j12 - j3 * j7 * j11);
	inverseJacobian[2][0] = (j6 * j12 - j7 * j11) / (j1 * j7 * j13 - j1 * j8 * j12 - j2 * j6 * j13 + j2 * j8 * j11 + j3 * j6 * j12 - j3 * j7 * j11);
	inverseJacobian[2][1] = -(j1 * j12 - j2 * j11) / (j1 * j7 * j13 - j1 * j8 * j12 - j2 * j6 * j13 + j2 * j8 * j11 + j3 * j6 * j12 - j3 * j7 * j11);
	inverseJacobian[2][2] = (j1 * j7 - j2 * j6) / (j1 * j7 * j13 - j1 * j8 * j12 - j2 * j6 * j13 + j2 * j8 * j11 + j3 * j6 * j12 - j3 * j7 * j11);

	constMatrix[0][0] =	(
								accJacobian[0][3] * x_dot_buildplate
							+	accJacobian[0][4] * y_dot_buildplate
							+	accJacobian[0][5] * z_dot_buildplate
							+	accJacobian[0][6] * theta_dot
							+	accJacobian[0][7] * phi_dot
						)
						+	invAccFrameMatrix[0][0]
						+
						(
								j4 * theta_ddot
							+	j5 * phi_ddot
						);
	constMatrix[1][0] = (
								accJacobian[1][3] * x_dot_buildplate
							+	accJacobian[1][4] * y_dot_buildplate
							+	accJacobian[1][5] * z_dot_buildplate
							+	accJacobian[1][6] * theta_dot
							+	accJacobian[1][7] * phi_dot
						)
						+	invAccFrameMatrix[1][0]
						+
						(
								j9 * theta_ddot
							+	j10 * phi_ddot
						);
	constMatrix[2][0] = (
								accJacobian[2][3] * x_dot_buildplate
							+	accJacobian[2][4] * y_dot_buildplate
							+	accJacobian[2][5] * z_dot_buildplate
							+	accJacobian[2][6] * theta_dot
							+	accJacobian[2][7] * phi_dot
						)
						+	invAccFrameMatrix[2][0]
						+
						(
								j14 * theta_ddot
							+	j15 * phi_ddot
						);

	r1 = acc_buildplate_00(constMatrix, inverseJacobian[0][0], inverseJacobian[0][1], inverseJacobian[0][2]);
	r2 = acc_buildplate_10(constMatrix, inverseJacobian[1][0], inverseJacobian[1][1], inverseJacobian[1][2]);
	r3 = acc_buildplate_20(constMatrix, inverseJacobian[2][0], inverseJacobian[2][1], inverseJacobian[2][2]);

	acc_buildplateMatrix[0][0] = r1;
	acc_buildplateMatrix[1][0] = r2;
	acc_buildplateMatrix[2][0] = r3;

	return acc_buildplateMatrix;
}

}