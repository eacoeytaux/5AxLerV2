#include "ForwardKinematics.hpp"
#include <math.h>

namespace cura {

float ForwardKinematics::pos_00(float x_frame, float x_buildplate, float y_buildplate,
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

	return		x_frame
			-	x_buildplate * (s_rho * s_theta - c_phi * c_rho * c_theta)
			-	z_offset * s_theta
			+	y_buildplate * (s_psi * commonExpr - c_psi * c_theta * s_phi)
			+	z_buildplate * (c_psi * commonExpr + c_theta * s_phi * s_psi);
}

float ForwardKinematics::pos_10(float y_frame, float x_buildplate, float y_buildplate,
	float z_buildplate, float rho, float phi, float psi) {
	float s_rho, s_phi, s_psi, c_rho, c_phi, c_psi;
	s_rho = sin(rho);
	s_phi = sin(phi);
	s_psi = sin(psi);
	c_rho = cos(rho);
	c_phi = cos(phi);
	c_psi = cos(psi);

	return		y_frame
			+	y_buildplate * (c_phi * c_psi + s_phi * s_psi * s_rho)
			-	z_buildplate * (c_phi * s_psi - c_psi * s_phi * s_rho)
			+	x_buildplate * c_rho * s_phi;
}

float ForwardKinematics::pos_20(float z_frame, float x_buildplate, float y_buildplate,
	float z_buildplate, float rho, float theta, float phi, float psi, float z_offset) {
	float s_rho, s_theta, s_phi, s_psi, c_rho, c_theta, c_phi, c_psi;
	s_rho = sin(rho);
	s_theta = sin(theta);
	s_phi = sin(phi);
	s_psi = sin(psi);
	c_rho = cos(rho);
	c_theta = cos(theta);
	c_phi = cos(phi);
	c_psi = cos(psi);

	return		z_offset + z_frame
			-	x_buildplate * (c_theta * s_rho - c_phi * c_rho * s_theta)
			-	z_offset * c_theta
			+	y_buildplate * (s_psi * (c_rho * c_theta - c_phi * s_rho * s_theta) + c_psi * s_phi * s_theta)
			+	z_buildplate * (c_psi * c_rho * c_theta - c_phi * s_rho * s_theta)
			-	s_phi * s_psi * s_theta;
}

float ForwardKinematics::vel_jacobian_03(float x_buildplate, float y_buildplate, float z_buildplate, float rho, float theta, float phi,
	float psi, float z_offset) { 
	float s_rho, s_theta, s_phi, s_psi, c_rho, c_theta, c_phi, c_psi;
	s_rho = sin(rho);
	s_theta = sin(theta);
	s_phi = sin(phi);
	s_psi = sin(psi);
	c_rho = cos(rho);
	c_theta = cos(theta);
	c_phi = cos(phi);
	c_psi = cos(psi);

	return 		y_buildplate * (s_psi * (c_rho * c_theta - c_phi * s_rho * s_theta) + c_psi * s_phi * s_theta)
			-	z_offset * c_theta
			-	x_buildplate * (c_theta * s_rho + c_phi * c_rho * s_theta)
			+	z_buildplate * (c_psi * (c_rho * c_theta - c_phi * s_rho * s_theta) - s_phi * s_psi * s_theta);
}

float ForwardKinematics::vel_jacobian_04(float x_buildplate, float y_buildplate, float z_buildplate, float rho, float theta, float phi,
	float psi) {
	float s_rho, s_phi, s_psi, c_rho, c_theta, c_phi, c_psi;
	s_rho = sin(rho);
	s_phi = sin(phi);
	s_psi = sin(psi);
	c_rho = cos(rho);
	c_theta = cos(theta);
	c_phi = cos(phi);
	c_psi = cos(psi);

	return		z_buildplate * (c_phi * c_theta * s_psi - c_psi * c_theta * s_phi * s_rho)
			-	y_buildplate* (c_phi * c_psi * c_theta + c_theta * s_phi * s_psi * s_rho)
			-	x_buildplate * c_rho * c_theta * s_phi;
}

float ForwardKinematics::vel_jacobian_05(float rho, float theta, float phi) {
	float s_rho, s_theta, c_rho, c_theta, c_phi;
	s_rho = sin(rho);
	s_theta = sin(theta);
	c_rho = cos(rho);
	c_theta = cos(theta);
	c_phi = cos(phi);

	return c_phi * c_rho * c_theta - s_rho * s_theta;
}

float ForwardKinematics::vel_jacobian_06(float rho, float theta, float phi, float psi) {
	float s_rho, s_theta, s_phi, s_psi, c_rho, c_theta, c_phi, c_psi;
	s_rho = sin(rho);
	s_theta = sin(theta);
	s_phi = sin(phi);
	s_psi = sin(psi);
	c_rho = cos(rho);
	c_theta = cos(theta);
	c_phi = cos(phi);
	c_psi = cos(psi);

	return s_psi * (c_rho * s_theta + c_phi * c_theta * s_rho) - c_psi * c_theta * s_phi;
}

float ForwardKinematics::vel_jacobian_07(float rho, float theta, float phi, float psi) {
	float s_rho, s_theta, s_phi, s_psi, c_rho, c_theta, c_phi, c_psi;
	s_rho = sin(rho);
	s_theta = sin(theta);
	s_phi = sin(phi);
	s_psi = sin(psi);
	c_rho = cos(rho);
	c_theta = cos(theta);
	c_phi = cos(phi);
	c_psi = cos(psi);

	return c_psi * (c_rho * s_theta + c_phi * c_theta * s_rho) + c_theta * s_phi * s_psi;
}

float ForwardKinematics::vel_jacobian_14(float x_buildplate, float y_buildplate, float z_buildplate, float rho, float phi, float psi) {
	float s_rho, s_phi, s_psi, c_rho, c_phi, c_psi;
	s_rho = sin(rho);
	s_phi = sin(phi);
	s_psi = sin(psi);
	c_rho = cos(rho);
	c_phi = cos(phi);
	c_psi = cos(psi);

	return 		z_buildplate * (s_phi * s_psi + c_phi * c_psi * s_rho)
			-	y_buildplate * (c_psi * s_phi - c_phi * s_psi * s_rho)
			+	x_buildplate * c_phi * c_rho;
}

float ForwardKinematics::vel_jacobian_15(float rho, float phi) {
	return cos(rho) * sin(phi);
}

float ForwardKinematics::vel_jacobian_16(float rho, float phi, float psi) {
	return cos(phi) * cos(psi) + sin(phi) * sin(psi) * sin(rho);
}

float ForwardKinematics::vel_jacobian_17(float rho, float phi, float psi) {
	return cos(psi) * sin(phi) * sin(rho) - cos(phi) * sin(psi);
}

float ForwardKinematics::vel_jacobian_23(float x_buildplate, float y_buildplate, float z_buildplate,
	float rho, float theta, float phi, float psi, float z_offset) {
	float s_rho, s_theta, s_phi, s_psi, c_rho, c_theta, c_phi, c_psi;
	s_rho = sin(rho);
	s_theta = sin(theta);
	s_phi = sin(phi);
	s_psi = sin(psi);
	c_rho = cos(rho);
	c_theta = cos(theta);
	c_phi = cos(phi);
	c_psi = cos(psi);

	return 		x_buildplate * (s_rho * s_theta - c_phi * c_rho * c_theta)
			+	z_offset * s_theta
			-	y_buildplate * (s_psi * (c_rho * s_theta + c_phi * c_theta * s_rho) - c_psi * c_theta * s_phi)
			-	z_buildplate * (c_psi * (c_rho * s_theta + c_phi * c_theta * s_rho) + c_theta * s_phi * s_psi);
}

float ForwardKinematics::vel_jacobian_24(float x_buildplate, float y_buildplate, float z_buildplate,
	float rho, float theta, float phi, float psi) {
	float s_rho, s_theta, s_phi, s_psi, c_rho, c_phi, c_psi;
	s_rho = sin(rho);
	s_theta = sin(theta);
	s_phi = sin(phi);
	s_psi = sin(psi);
	c_rho = cos(rho);
	c_phi = cos(phi);
	c_psi = cos(psi);

	return		y_buildplate * (c_phi * c_psi * s_theta + s_phi * s_psi * s_rho * s_theta)
			-	z_buildplate * (c_phi * s_psi * s_theta - c_psi * s_phi * s_rho * s_theta)
			+	x_buildplate * c_rho * s_phi * s_theta;
}

float ForwardKinematics::vel_jacobian_25(float rho, float theta, float phi) {
	return -cos(theta) * sin(rho) - cos(phi) * cos(rho) * sin(theta);
}

float ForwardKinematics::vel_jacobian_26(float rho, float theta, float phi, float psi) {
	float s_rho, s_theta, s_phi, s_psi, c_rho, c_theta, c_phi, c_psi;
	s_rho = sin(rho);
	s_theta = sin(theta);
	s_phi = sin(phi);
	s_psi = sin(psi);
	c_rho = cos(rho);
	c_theta = cos(theta);
	c_phi = cos(phi);
	c_psi = cos(psi);

	return 		s_psi * (c_rho * c_theta - c_phi * s_rho * s_theta)
			+	c_psi * s_phi * s_theta;
}

float ForwardKinematics::vel_jacobian_27(float rho, float theta, float phi, float psi) {
	float s_rho, s_theta, s_phi, s_psi, c_rho, c_theta, c_phi, c_psi;
	s_rho = sin(rho);
	s_theta = sin(theta);
	s_phi = sin(phi);
	s_psi = sin(psi);
	c_rho = cos(rho);
	c_theta = cos(theta);
	c_phi = cos(phi);
	c_psi = cos(psi);

	return 		c_psi * (c_rho * c_theta - c_phi * s_rho * s_theta)
			+	s_phi * s_psi * s_theta;
}

float ForwardKinematics::acc_jacobian_03(float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate, float theta_dot,
	float phi_dot,float x_buildplate, float y_buildplate, float z_buildplate, float rho, float theta,
	float phi, float psi, float z_offset) {
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

	return		theta_dot * (x_buildplate * (s_rho * s_theta - c_phi * c_rho * c_theta) + z_offset * s_theta - y_buildplate * (s_psi * commonExpr - c_psi * c_theta * s_phi) - (z_buildplate * (c_psi * commonExpr + c_theta * s_phi * s_psi)))
			-	x_dot_buildplate * (c_theta * s_rho + c_phi * c_rho * s_theta)
			+	phi_dot * (y_buildplate * (c_phi * c_psi * s_theta + s_phi * s_psi * s_rho * s_theta) - z_buildplate * (c_phi * s_psi * s_theta - c_psi * s_phi * s_rho * s_theta) + x_buildplate * c_rho * s_phi * s_theta)
			+	y_dot_buildplate * (s_psi * (c_rho * c_theta - c_phi * s_rho * s_theta) + c_psi * s_phi * s_theta)
			+	z_dot_buildplate * (c_psi * (c_rho * c_theta - c_psi * s_rho * s_theta) - s_phi * s_psi * s_theta);
}

float ForwardKinematics::acc_jacobian_04(float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate, float theta_dot,
	float phi_dot,float x_buildplate, float y_buildplate, float z_buildplate, float rho, float theta,
	float phi, float psi) {
	float s_rho, s_theta, s_phi, s_psi, c_rho, c_theta, c_phi, c_psi;
	s_rho = sin(rho);
	s_theta = sin(theta);
	s_phi = sin(phi);
	s_psi = sin(psi);
	c_rho = cos(rho);
	c_theta = cos(theta);
	c_phi = cos(phi);
	c_psi = cos(psi);

	return		theta_dot * (y_buildplate * (c_phi * c_psi * s_theta + s_phi * s_psi * s_rho * s_theta) - z_buildplate * (c_phi * s_psi * s_theta - c_psi * s_phi * s_rho * s_theta) + x_buildplate * c_rho * s_phi * s_theta)
			-	phi_dot * (z_buildplate * (c_theta * s_phi * s_psi + c_phi * c_psi * c_theta * s_rho) - y_buildplate * (c_psi * c_theta * s_phi - c_phi * c_theta * s_psi * s_rho) + x_buildplate * c_phi * c_rho * c_theta)
			-	y_dot_buildplate * (c_phi * c_psi * c_theta + c_theta * s_phi * s_psi * s_rho)
			+	z_dot_buildplate * (c_phi * c_theta * s_psi - c_psi * c_theta * s_phi * s_rho)
			-	x_dot_buildplate * c_rho * c_theta * s_phi;
}

float ForwardKinematics::acc_jacobian_05(float theta_dot, float phi_dot, float rho, float theta, float phi) {
	float s_rho, s_theta, s_phi, c_rho, c_theta, c_phi;
	s_rho = sin(rho);
	s_theta = sin(theta);
	s_phi = sin(phi);
	c_rho = cos(rho);
	c_theta = cos(theta);
	c_phi = cos(phi);

	return	-	theta_dot * (c_theta * s_rho + c_phi * c_rho * s_theta)
			-	phi_dot * c_rho * c_theta * s_phi;
}

float ForwardKinematics::acc_jacobian_06(float theta_dot, float phi_dot, float rho, float theta, float phi, float psi) {
	float s_rho, s_theta, s_phi, s_psi, c_rho, c_theta, c_phi, c_psi;
	s_rho = sin(rho);
	s_theta = sin(theta);
	s_phi = sin(phi);
	s_psi = sin(psi);
	c_rho = cos(rho);
	c_theta = cos(theta);
	c_phi = cos(phi);
	c_psi = cos(psi);

	return		theta_dot * (s_psi * (c_rho * c_theta - c_phi * s_rho * s_theta) + c_psi * s_phi * s_theta)
			-	phi_dot * (c_phi * c_psi * c_theta + c_theta * s_phi * s_psi * s_rho);
}

float ForwardKinematics::acc_jacobian_07(float theta_dot, float phi_dot, float rho, float theta, float phi, float psi) {
	float s_rho, s_theta, s_phi, s_psi, c_rho, c_theta, c_phi, c_psi;
	s_rho = sin(rho);
	s_theta = sin(theta);
	s_phi = sin(phi);
	s_psi = sin(psi);
	c_rho = cos(rho);
	c_theta = cos(theta);
	c_phi = cos(phi);
	c_psi = cos(psi);

	return		theta_dot * (c_psi * (c_rho * c_theta - c_phi * s_rho * s_theta) - s_phi * s_psi * s_theta)
			+	phi_dot * (c_phi * c_theta * s_psi - c_psi * c_theta * s_phi * s_rho);
}

float ForwardKinematics::acc_jacobian_14(float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate, float phi_dot,
	float x_buildplate, float y_buildplate, float z_buildplate, float rho, float phi, float psi) {
	float s_rho, s_phi, s_psi, c_rho, c_phi, c_psi;
	s_rho = sin(rho);
	s_phi = sin(phi);
	s_psi = sin(psi);
	c_rho = cos(rho);
	c_phi = cos(phi);
	c_psi = cos(psi);

	return		z_dot_buildplate * (s_phi * s_psi + c_phi * c_psi * s_rho)
			-	y_dot_buildplate * (c_psi * s_phi - c_phi * s_psi * s_rho)
			-	phi_dot * (y_buildplate * (c_phi * c_psi + s_phi * s_psi * s_rho) - z_buildplate * (c_phi * s_psi - c_psi * s_phi * s_rho) + x_buildplate * c_rho * s_phi)
			+	x_dot_buildplate * c_phi * c_rho;
}

float ForwardKinematics::acc_jacobian_15(float phi_dot, float rho, float phi) {
	return phi_dot * cos(phi) * cos(rho);
}

float ForwardKinematics::acc_jacobian_16(float phi_dot, float rho, float phi, float psi) {
	return -phi_dot * (cos(psi) * sin(phi) - cos(phi) * sin(psi) * sin(rho));
}

float ForwardKinematics::acc_jacobian_17(float phi_dot, float rho, float phi, float psi) {
	return phi_dot * (sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(rho));
}

float ForwardKinematics::acc_jacobian_23(float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate, float theta_dot,
	float phi_dot,float x_buildplate, float y_buildplate, float z_buildplate, float rho, float theta,
	float phi, float psi) {
	float s_rho, s_theta, s_phi, s_psi, c_rho, c_theta, c_phi, c_psi;
	s_rho = sin(rho);
	s_theta = sin(theta);
	s_phi = sin(phi);
	s_psi = sin(psi);
	c_rho = cos(rho);
	c_theta = cos(theta);
	c_phi = cos(phi);
	c_psi = cos(psi);

	return		x_dot_buildplate * (s_rho * s_theta - c_phi * c_rho * c_theta)
			+	phi_dot * (y_buildplate * (c_phi * c_psi * c_theta + c_theta * s_phi * s_psi * s_rho) - z_buildplate * (c_phi * c_theta * s_psi - c_psi * c_theta * s_phi * s_rho) + x_buildplate * c_rho * c_theta * s_phi)
			-	y_dot_buildplate * (s_psi * (c_rho * s_theta + c_phi * c_theta * s_rho) - c_psi * c_theta * s_phi);
}

float ForwardKinematics::acc_jacobian_24(float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate, float theta_dot,
	float phi_dot,float x_buildplate, float y_buildplate, float z_buildplate, float rho, float theta,
	float phi, float psi) {
	float s_rho, s_theta, s_phi, s_psi, c_rho, c_theta, c_phi, c_psi;
	s_rho = sin(rho);
	s_theta = sin(theta);
	s_phi = sin(phi);
	s_psi = sin(psi);
	c_rho = cos(rho);
	c_theta = cos(theta);
	c_phi = cos(phi);
	c_psi = cos(psi);

	return		y_dot_buildplate * (c_phi * c_psi * s_theta + s_phi * s_psi * s_rho * s_theta)
			-	z_dot_buildplate * (c_phi * s_psi * s_theta - c_psi * s_phi * s_rho * s_theta)
			+	theta_dot * (y_buildplate * (c_phi * c_psi * c_theta + c_theta * s_phi * s_psi * s_rho) - z_buildplate * (c_phi * c_theta * s_psi - c_psi * c_theta * s_phi * s_rho) + x_buildplate * c_rho * c_theta * s_phi)
			+	phi_dot * (z_buildplate * (s_phi * s_psi * s_theta + c_phi * c_psi * s_rho * s_theta) - y_buildplate * (c_psi * s_phi * s_theta - c_phi * s_psi * s_rho * s_theta) + x_buildplate * c_phi * c_rho * s_theta)
			+	x_dot_buildplate * c_rho * s_phi * s_theta;
}

float ForwardKinematics::acc_jacobian_25(float theta_dot, float phi_dot, float rho, float theta, float phi) {
	float s_rho, s_theta, s_phi, c_rho, c_theta, c_phi;
	s_rho = sin(rho);
	s_theta = sin(theta);
	s_phi = sin(phi);
	c_rho = cos(rho);
	c_theta = cos(theta);
	c_phi = cos(phi);

	return		theta_dot * (s_rho * s_theta - c_phi * c_rho * c_theta)
			+	phi_dot * c_rho * s_phi * s_theta;
}

float ForwardKinematics::acc_jacobian_26(float theta_dot, float phi_dot, float rho, float theta, float phi, float psi) {
	float s_rho, s_theta, s_phi, s_psi, c_rho, c_theta, c_phi, c_psi;
	s_rho = sin(rho);
	s_theta = sin(theta);
	s_phi = sin(phi);
	s_psi = sin(psi);
	c_rho = cos(rho);
	c_theta = cos(theta);
	c_phi = cos(phi);
	c_psi = cos(psi);

	return		phi_dot * (c_phi * c_psi * s_theta + s_phi * s_psi * s_rho * s_theta)
			-	theta_dot * (s_psi * (c_rho * s_theta + c_phi * c_theta * s_rho) - c_psi * c_theta * s_phi);
}

float ForwardKinematics::acc_jacobian_27(float theta_dot, float phi_dot, float rho, float theta, float phi, float psi) {
	float s_rho, s_theta, s_phi, s_psi, c_rho, c_theta, c_phi, c_psi;
	s_rho = sin(rho);
	s_theta = sin(theta);
	s_phi = sin(phi);
	s_psi = sin(psi);
	c_rho = cos(rho);
	c_theta = cos(theta);
	c_phi = cos(phi);
	c_psi = cos(psi);

	return		-	phi_dot * (c_phi * s_psi * s_theta - c_psi * s_phi * s_rho * s_theta)
				-	theta_dot * (c_psi * (c_rho * s_theta + c_phi * c_theta * s_rho) + c_theta * s_phi * s_psi);
}

Matrix3x1 ForwardKinematics::position(float x_frame, float y_frame, float z_frame, float x_buildplate, float y_buildplate,
	float z_buildplate, float rho, float theta, float phi, float psi, float z_offset) {
	float r1, r2, r3;
	
	r1 = pos_00(x_frame, x_buildplate, y_buildplate, z_buildplate, rho, theta, phi, psi, z_offset);
	r2 = pos_10(y_frame, x_buildplate, y_buildplate, z_buildplate, rho, phi, psi);
	r3 = pos_20(z_frame, x_buildplate, y_buildplate, z_buildplate, rho, theta, phi, psi, z_offset);

	posMatrix[0][0] = r1;
	posMatrix[1][0] = r2;
	posMatrix[2][0] = r3;

	return posMatrix;
}

Matrix3x8 ForwardKinematics::velocity_jacobian(float x_buildplate, float y_buildplate, float z_buildplate,
	float rho, float theta, float phi, float psi, float z_offset) {

	velMatrix[0][0] = vel_jacobian_00();
	velMatrix[0][1] = vel_jacobian_01();
	velMatrix[0][2] = vel_jacobian_02();
	velMatrix[0][3] = vel_jacobian_03(x_buildplate, y_buildplate, z_buildplate, rho, theta, phi, psi, z_offset);
	velMatrix[0][4] = vel_jacobian_04(x_buildplate, y_buildplate, z_buildplate, rho, theta, phi, psi);
	velMatrix[0][5] = vel_jacobian_05(rho, theta, phi);
	velMatrix[0][6] = vel_jacobian_06(rho, theta, phi, psi);
	velMatrix[0][7] = vel_jacobian_07(rho, theta, phi, psi);

	velMatrix[1][0] = vel_jacobian_10();
	velMatrix[1][1] = vel_jacobian_11();
	velMatrix[1][2] = vel_jacobian_12();
	velMatrix[1][3] = vel_jacobian_13();
	velMatrix[1][4] = vel_jacobian_14(x_buildplate, y_buildplate, z_buildplate, rho, phi, psi);
	velMatrix[1][5] = vel_jacobian_15(rho, phi);
	velMatrix[1][6] = vel_jacobian_16(rho, phi, psi);
	velMatrix[1][7] = vel_jacobian_17(rho, phi, psi);

	velMatrix[2][0] = vel_jacobian_20();
	velMatrix[2][1] = vel_jacobian_21();
	velMatrix[2][2] = vel_jacobian_22();
	velMatrix[2][3] = vel_jacobian_23(x_buildplate, y_buildplate, z_buildplate, rho, theta, phi, psi, z_offset);
	velMatrix[2][4] = vel_jacobian_24(x_buildplate, y_buildplate, z_buildplate, rho, theta, phi, psi);
	velMatrix[2][5] = vel_jacobian_25(rho, theta, phi);
	velMatrix[2][6] = vel_jacobian_26(rho, theta, phi, psi);
	velMatrix[2][7] = vel_jacobian_27(rho, theta, phi, psi);

	return velMatrix;
}

Matrix3x8 ForwardKinematics::acceleration_jacobian(float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate,
	float theta_dot, float phi_dot, float x_buildplate, float y_buildplate, float z_buildplate,
	float rho, float theta, float phi, float psi, float z_offset) {

	accMatrix[0][0] = acc_jacobian_00();
	accMatrix[0][1] = acc_jacobian_01();
	accMatrix[0][2] = acc_jacobian_02();
	accMatrix[0][3] = acc_jacobian_03(x_dot_buildplate, y_dot_buildplate, z_dot_buildplate, theta_dot, phi_dot,x_buildplate,
		y_buildplate, z_buildplate, rho, theta, phi, psi, z_offset);
	accMatrix[0][4] = acc_jacobian_04(x_dot_buildplate, y_dot_buildplate, z_dot_buildplate, theta_dot, phi_dot, x_buildplate,
		y_buildplate, z_buildplate, rho, theta, phi, psi);
	accMatrix[0][5] = acc_jacobian_05(theta_dot, phi_dot, rho, theta, phi);
	accMatrix[0][6] = acc_jacobian_06(theta_dot, phi_dot, rho, theta, phi, psi);
	accMatrix[0][7] = acc_jacobian_07(theta_dot, phi_dot, rho, theta, phi, psi);

	accMatrix[1][0] = acc_jacobian_10();
	accMatrix[1][1] = acc_jacobian_11();
	accMatrix[1][2] = acc_jacobian_12();
	accMatrix[1][3] = acc_jacobian_13();
	accMatrix[1][4] = acc_jacobian_14(x_dot_buildplate, y_dot_buildplate, z_dot_buildplate, phi_dot, x_buildplate, y_buildplate,
		z_buildplate, rho, phi, psi);
	accMatrix[1][5] = acc_jacobian_15(phi_dot, rho, phi);
	accMatrix[1][6] = acc_jacobian_16(phi_dot, rho, phi, psi);
	accMatrix[1][7] = acc_jacobian_17(phi_dot, rho, phi, psi);

	accMatrix[2][0] = acc_jacobian_20();
	accMatrix[2][1] = acc_jacobian_21();
	accMatrix[2][2] = acc_jacobian_22();
	accMatrix[2][3] = acc_jacobian_23(x_dot_buildplate, y_dot_buildplate, z_dot_buildplate, theta_dot, phi_dot,x_buildplate,
		y_buildplate, z_buildplate, rho, theta, phi, psi);
	accMatrix[2][4] = acc_jacobian_24(x_dot_buildplate, y_dot_buildplate, z_dot_buildplate, theta_dot, phi_dot,x_buildplate,
		y_buildplate, z_buildplate, rho, theta, phi, psi);
	accMatrix[2][5] = acc_jacobian_25(theta_dot, phi_dot, rho, theta, phi);
	accMatrix[2][6] = acc_jacobian_26(theta_dot, phi_dot, rho, theta, phi, psi);
	accMatrix[2][7] = acc_jacobian_27(theta_dot, phi_dot, rho, theta, phi, psi);

	return accMatrix;
}

}