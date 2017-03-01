#include "InverseKinematics.hpp"

namespace cura {

float InverseKinematics::pos_row1(float x_S, float x_buildplate, float y_buildplate,
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

	float retVal = 	(x_S) -
					(	
						(x_buildplate * (s_rho * s_theta - c_phi * c_rho * c_theta)) -
						(z_offset * s_theta) +
						(y_buildplate * (s_psi * commonExpr - c_psi * c_theta * s_phi)) +
						(z_buildplate * (c_psi * commonExpr + c_theta * s_phi * s_psi))
					);

	return retVal;
}

float InverseKinematics::pos_row2(float y_S, float x_buildplate, float y_buildplate,
	float z_buildplate, float rho, float phi, float psi) {
	// Pre-compute trigonometric values
	float s_rho, s_phi, s_psi, c_rho, c_phi, c_psi;
	s_rho = sin(rho);
	s_phi = sin(phi);
	s_psi = sin(psi);
	c_rho = cos(rho);
	c_phi = cos(phi);
	c_psi = cos(psi);

	float retVal =	(y_S) -
					(
						(y_buildplate * (c_phi * c_psi + s_phi * s_psi * s_rho)) -
						(z_buildplate * (c_phi * s_psi - c_psi * s_phi * s_rho)) +
						(x_buildplate * c_rho * s_phi)
					);

	return retVal;
}

float InverseKinematics::pos_row3(float z_S, float x_buildplate, float y_buildplate,
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

	float retVal = 	(z_S) -
					((z_offset) -
					(x_buildplate * (c_theta * s_rho - c_phi * c_rho * s_theta)) -
					(z_offset * c_theta) +
					(y_buildplate * (s_psi * (c_rho * c_theta - c_phi * s_rho * s_theta) + c_psi * s_phi * s_theta)) +
					(z_buildplate * (c_psi * c_rho * c_theta - c_phi * s_rho * s_theta)) -
					(s_phi * s_psi * s_theta));

	return retVal;
}

Matrix3x1 InverseKinematics::position(float x_S, float y_S, float z_S, float x_buildplate, float y_buildplate,
		float z_buildplate, float rho, float theta, float phi, float psi, float z_offset) {
	float r1, r2, r3;

	r1 = pos_row1(x_S, x_buildplate, y_buildplate, z_buildplate, rho, theta, phi, psi, z_offset);
	r2 = pos_row2(y_S, x_buildplate, y_buildplate, z_buildplate, rho, phi, psi);
	r3 = pos_row3(z_S, x_buildplate, y_buildplate, z_buildplate, rho, theta, phi, psi, z_offset);

	posMatrix[0][0] = r1;
	posMatrix[1][0] = r2;
	posMatrix[2][0] = r3;

	return posMatrix;
}

}