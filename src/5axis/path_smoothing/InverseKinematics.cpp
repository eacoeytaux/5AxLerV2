#include "InverseKinematics.hpp"

namespace cura {

float InverseKinematics::pos_00(float x_S, float x_buildplate, float y_buildplate,
	float z_buildplate, float rho, float theta, float phi, float psi, float z_offset) {
	// Pre-compute trigonometric values
	float s_rho, s_theta, s_phi, s_psi, c_rho, c_theta, c_phi, c_psi, commonExpr;
	s_rho = sin(rho);
	s_theta = sin(theta);
	s_phi = sin(phi);
	s_psi = sin(psi);
	c_rho = cos(rho);
	c_theta = cos(theta);
	c_phi = cos(phi);
	c_psi = cos(psi);
	commonExpr = (c_rho * s_theta + c_phi * c_theta * s_rho);

	return		x_S
			-	(	
					  x_buildplate * (s_rho * s_theta - c_phi * c_rho * c_theta)
					- z_offset * s_theta
					+ y_buildplate * (s_psi * commonExpr - c_psi * c_theta * s_phi)
					+ z_buildplate * (c_psi * commonExpr + c_theta * s_phi * s_psi)
				);
}

float InverseKinematics::pos_10(float y_S, float x_buildplate, float y_buildplate,
	float z_buildplate, float rho, float phi, float psi) {
	// Pre-compute trigonometric values
	float s_rho, s_phi, s_psi, c_rho, c_phi, c_psi;
	s_rho = sin(rho);
	s_phi = sin(phi);
	s_psi = sin(psi);
	c_rho = cos(rho);
	c_phi = cos(phi);
	c_psi = cos(psi);

	return		y_S
			-	(
					  y_buildplate * (c_phi * c_psi + s_phi * s_psi * s_rho)
					- z_buildplate * (c_phi * s_psi - c_psi * s_phi * s_rho)
					+ x_buildplate * c_rho * s_phi
				);
}

float InverseKinematics::pos_20(float z_S, float x_buildplate, float y_buildplate,
	float z_buildplate, float rho, float theta, float phi, float psi, float z_offset) {
	float s_rho, s_theta, s_phi, s_psi, c_rho, c_theta, c_phi, c_psi;
	// Pre-compute trigonometric values
	s_rho = sin(rho);
	s_theta = sin(theta);
	s_phi = sin(phi);
	s_psi = sin(psi);
	c_rho = cos(rho);
	c_theta = cos(theta);
	c_phi = cos(phi);
	c_psi = cos(psi);

	return		z_S
			-	(
					  z_offset
					- x_buildplate * (c_theta * s_rho - c_phi * c_rho * s_theta)
					- z_offset * c_theta
					+ y_buildplate * (s_psi * (c_rho * c_theta - c_phi * s_rho * s_theta) + c_psi * s_phi * s_theta)
					+ z_buildplate * (c_psi * c_rho * c_theta - c_phi * s_rho * s_theta)
					- s_phi * s_psi * s_theta
				);
}

float InverseKinematics::vel_00(float forwVel_00, float forwVel_01, float forwVel_02, float forwVel_03, float forwVel_04,
	float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate, float theta_dot, float phi_dot) {
	return	-	(
					  forwVel_00 * x_dot_buildplate
					+ forwVel_01 * y_dot_buildplate
					+ forwVel_02 * z_dot_buildplate
					+ forwVel_03 * theta_dot
					+ forwVel_04 * phi_dot
				);
}

float InverseKinematics::vel_10(float forwVel_10, float forwVel_11, float forwVel_12, float forwVel_13, float forwVel_14,
	float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate, float theta_dot, float phi_dot) {
	return	-	(
					  forwVel_10 * x_dot_buildplate
					+ forwVel_11 * y_dot_buildplate
					+ forwVel_12 * z_dot_buildplate
					+ forwVel_13 * theta_dot
					+ forwVel_14 * phi_dot
				);
}

float InverseKinematics::vel_20(float forwVel_20, float forwVel_21, float forwVel_22, float forwVel_23, float forwVel_24,
	float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate, float theta_dot, float phi_dot) {
	return	-	(
					  forwVel_20 * x_dot_buildplate
					+ forwVel_21 * y_dot_buildplate
					+ forwVel_22 * z_dot_buildplate
					+ forwVel_23 * theta_dot
					+ forwVel_24 * phi_dot
				);
}

float InverseKinematics::acc_00(float forwAcc_00, float forwAcc_01, float forwAcc_02, float forwAcc_03, float forwAcc_04,
	float forwVel_00, float forwVel_01, float forwVel_02, float forwVel_03, float forwVel_04,
	float x_ddot_buildplate, float y_ddot_buildplate, float z_ddot_buildplate, float theta_ddot, float phi_ddot,
	float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate, float theta_dot, float phi_dot) {
	float velComponent =		forwAcc_00 * x_dot_buildplate
							+	forwAcc_01 * y_dot_buildplate
							+	forwAcc_02 * z_dot_buildplate
							+	forwAcc_03 * theta_dot
							+	forwAcc_04 * phi_dot;

	float accComponent =		forwVel_00 * x_ddot_buildplate
							+	forwVel_01 * y_ddot_buildplate
							+	forwVel_02 * z_ddot_buildplate
							+	forwVel_03 * theta_ddot
							+	forwVel_04 * phi_ddot;

	return -velComponent - accComponent;
}

float InverseKinematics::acc_10(float forwAcc_10, float forwAcc_11, float forwAcc_12, float forwAcc_13, float forwAcc_14,
	float forwVel_10, float forwVel_11, float forwVel_12, float forwVel_13, float forwVel_14,
	float x_ddot_buildplate, float y_ddot_buildplate, float z_ddot_buildplate, float theta_ddot, float phi_ddot,
	float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate, float theta_dot, float phi_dot) {
	float velComponent =		forwAcc_10 * x_dot_buildplate
							+	forwAcc_11 * y_dot_buildplate
							+	forwAcc_12 * z_dot_buildplate
							+	forwAcc_13 * theta_dot
							+	forwAcc_14 * phi_dot;

	float accComponent =		forwVel_10 * x_ddot_buildplate
							+	forwVel_11 * y_ddot_buildplate
							+	forwVel_12 * z_ddot_buildplate
							+	forwVel_13 * theta_ddot
							+	forwVel_14 * phi_ddot;

	return -velComponent - accComponent;
}

float InverseKinematics::acc_20(float forwAcc_20, float forwAcc_21, float forwAcc_22, float forwAcc_23, float forwAcc_24,
		float forwVel_20, float forwVel_21, float forwVel_22, float forwVel_23, float forwVel_24,
		float x_ddot_buildplate, float y_ddot_buildplate, float z_ddot_buildplate, float theta_ddot, float phi_ddot,
		float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate, float theta_dot, float phi_dot) {
	float velComponent =		forwAcc_20 * x_dot_buildplate
							+	forwAcc_21 * y_dot_buildplate
							+	forwAcc_22 * z_dot_buildplate
							+	forwAcc_23 * theta_dot
							+	forwAcc_24 * phi_dot;

	float accComponent =		forwVel_20 * x_ddot_buildplate
							+	forwVel_21 * y_ddot_buildplate
							+	forwVel_22 * z_ddot_buildplate
							+	forwVel_23 * theta_ddot
							+	forwVel_24 * phi_ddot;

	return -velComponent - accComponent;
}

Matrix3x1 InverseKinematics::position(float x_S, float y_S, float z_S, float x_buildplate, float y_buildplate,
		float z_buildplate, float rho, float theta, float phi, float psi, float z_offset) {
	float r1, r2, r3;
	r1 = pos_00(x_S, x_buildplate, y_buildplate, z_buildplate, rho, theta, phi, psi, z_offset);
	r2 = pos_10(y_S, x_buildplate, y_buildplate, z_buildplate, rho, phi, psi);
	r3 = pos_20(z_S, x_buildplate, y_buildplate, z_buildplate, rho, theta, phi, psi, z_offset);

	posMatrix[0][0] = r1;
	posMatrix[1][0] = r2;
	posMatrix[2][0] = r3;

	return posMatrix;
}

Matrix3x1 InverseKinematics::velocity(Matrix3x8 forwVelMatrix, float x_dot_buildplate, float y_dot_buildplate,
	float z_dot_buildplate, float theta_dot, float phi_dot) {
	float r1, r2, r3;
	r1 = vel_00(forwVelMatrix[0][3], forwVelMatrix[0][4], forwVelMatrix[0][5], forwVelMatrix[0][6],
		forwVelMatrix[0][7], x_dot_buildplate, y_dot_buildplate, z_dot_buildplate, theta_dot, phi_dot);
	r2 = vel_10(forwVelMatrix[1][3], forwVelMatrix[1][4], forwVelMatrix[1][5], forwVelMatrix[1][6],
		forwVelMatrix[1][7], x_dot_buildplate, y_dot_buildplate, z_dot_buildplate, theta_dot, phi_dot);
	r3 = vel_20(forwVelMatrix[2][3], forwVelMatrix[2][4], forwVelMatrix[2][5], forwVelMatrix[2][6],
		forwVelMatrix[2][7], x_dot_buildplate, y_dot_buildplate, z_dot_buildplate, theta_dot, phi_dot);
	
	velMatrix[0][0] = r1;
	velMatrix[1][0] = r2;
	velMatrix[2][0] = r3;

	return velMatrix;
}

Matrix3x1 InverseKinematics::acceleration(Matrix3x8 forwAccMatrix, Matrix3x8 forwVelMatrix, float x_ddot_buildplate,
		float y_ddot_buildplate, float z_ddot_buildplate, float theta_ddot, float phi_ddot, 
		float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate, float theta_dot, float phi_dot) {
	float r1, r2, r3;
	r1 = acc_00(forwAccMatrix[0][3], forwAccMatrix[0][4], forwAccMatrix[0][5], forwAccMatrix[0][6], forwAccMatrix[0][7],
		forwVelMatrix[0][3], forwVelMatrix[0][4], forwVelMatrix[0][5], forwVelMatrix[0][6], forwVelMatrix[0][7],
		x_ddot_buildplate, y_ddot_buildplate, z_ddot_buildplate, theta_ddot, phi_ddot,
		x_dot_buildplate, y_dot_buildplate, z_dot_buildplate, theta_dot, phi_dot);
	r2 = acc_10(forwAccMatrix[1][3], forwAccMatrix[1][4], forwAccMatrix[1][5], forwAccMatrix[1][6], forwAccMatrix[1][7],
		forwVelMatrix[1][3], forwVelMatrix[1][4], forwVelMatrix[1][5], forwVelMatrix[1][6], forwVelMatrix[1][7],
		x_ddot_buildplate, y_ddot_buildplate, z_ddot_buildplate, theta_ddot, phi_ddot,
		x_dot_buildplate, y_dot_buildplate, z_dot_buildplate, theta_dot, phi_dot);
	r3 = acc_20(forwAccMatrix[2][3], forwAccMatrix[2][4], forwAccMatrix[2][5], forwAccMatrix[2][6], forwAccMatrix[2][7],
		forwVelMatrix[2][3], forwVelMatrix[2][4], forwVelMatrix[2][5], forwVelMatrix[2][6], forwVelMatrix[2][7],
		x_ddot_buildplate, y_ddot_buildplate, z_ddot_buildplate, theta_ddot, phi_ddot,
		x_dot_buildplate, y_dot_buildplate, z_dot_buildplate, theta_dot, phi_dot);
	
	accMatrix[0][0] = r1;
	accMatrix[1][0] = r2;
	accMatrix[2][0] = r3;

	return accMatrix;
}

}