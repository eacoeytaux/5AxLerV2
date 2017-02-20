#include "PathSmoother.hpp"
#include "../../utils/logoutput.h"
#include <iostream>
#include <fstream>

namespace cura {

PathSmoother::PathSmoother(char* gcodeFilePath) {
	// Set the initial value of x/y/z point to be 0, 0, 0
	lastX = lastY = lastZ = 0;

	// Set the initial value of the current position to home values
	currX = X_HOME;
	currY = Y_HOME;
	currZ = Z_HOME;

	// Open the GCode file
	std::ifstream file(gcodeFilePath);

	// Check we successfully opened
	if (file.is_open()) {
		// Process each line individually
		std::string line;
		while (getline(file, line)) {
			// Only bother processing if the line is non-blank
			if (line.empty()) continue;

			// Check to see what type of line it is
			switch (line[0]) {
				// Comment, used to separate GCode layers
				case ';':
					if (line.substr(1, 6).compare("LAYER:") == 0) {
						int layer_nr = std::stoi(line.substr(7));
						
						if (layer_nr > 0) {
							processLayer();
							layerCommands.clear();
							getchar();
						}
					}
					break;

				// M-command
				case 'M':
					break;

				// G-command
				case 'G':
					processGCommand(line);
					break;

				default:
					break;
			}
		}

		// Close the file handler
		file.close();
	} else {
		logAlways("could not open file\n");
	}
}

void PathSmoother::processLayer() {
	// Get the shortest distance from the first 15 path segments
	FPoint3 maxAccel_vect = FPoint3(MAX_ACCEL, MAX_ACCEL, MAX_ACCEL);
	float maxFeedrate = findMinFeedrate(layerPoints, 0, 15, maxAccel_vect);

	logAlways("[INFO] maxFeedrate = %f\n", maxFeedrate);

	// Iterate through each point on the path
	// for (unsigned int point_idx = 0; point_idx < layerPoints.size() - 2; ++point_idx) {
	// 	FPoint3 p1 = layerPoints[point_idx];
	// 	FPoint3 p2 = layerPoints[point_idx + 1];
	// 	FPoint3 p3 = layerPoints[point_idx + 2];

	// }
}

void PathSmoother::createSpline(FPoint3 p1, FPoint3 p2, FPoint3 p3, FPoint3 accelProfile, float feedrate) {
	FPoint3 lineSegment1 = (p2 - p1);
	FPoint3 lineSegment2 = (p3 - p2);
	FPoint3 lineSegment1Normalized = lineSegment1.normalized();
	FPoint3 lineSegment2Normalized = lineSegment2.normalized();

	float angle = acos((lineSegment1Normalized.x * lineSegment2Normalized.x + lineSegment1Normalized.y * lineSegment2Normalized.y + lineSegment1Normalized.z * lineSegment2Normalized.z));

	FPoint3 v1_vect = lineSegment1Normalized * feedrate;
	FPoint3 v2_vect = lineSegment2Normalized * feedrate;

	FPoint3 a_vect = (accelProfile * 4) / pow(feedrate, 2);
	FPoint3 b_vect = lineSegment1Normalized - lineSegment2Normalized;

	float root_x = -b_vect.x / a_vect.x;
	float root_y = -b_vect.y / a_vect.y;
	float root_z = -b_vect.z / a_vect.z;
	float delta = root_x;
	if (root_y > delta) delta = root_y;
	if (root_z > delta) delta = root_z;

	if (delta > 0 && angle > MAX_ANGLE) {
		logAlways("=== INFO ===\n");
		logAlways("pi = <%f, %f, %f>, pm = <%f, %f, %f>, pf = <%f, %f, %f>\n", p1.x, p1.y, p1.z, p2.x, p2.y, p2.z, p3.x, p3.y, p3.z);
		logAlways("ls1n = <%f, %f, %f>, ls2n = <%f, %f, %f>\n", lineSegment1Normalized.x, lineSegment1Normalized.y, lineSegment1Normalized.z, lineSegment2Normalized.x, lineSegment1Normalized.y, lineSegment2Normalized.z);
		logAlways("v1_vect = <%f, %f, %f>, v2_vect = <%f, %f, %f>\n", v1_vect.x, v1_vect.y, v1_vect.z, v2_vect.x, v2_vect.y, v2_vect.z);
		logAlways("a_vect = <%f, %f, %f>, b_vect = <%f, %f, %f>\n", a_vect.x, a_vect.y, a_vect.z, b_vect.x, b_vect.y, b_vect.z);
		logAlways("root_x = %f, root_y = %f, root_z = %f\n", root_x, root_y, root_z);

		FPoint3 splineEndPoints[2];
		splineEndPoints[0] = p2 - lineSegment1Normalized * delta;
		splineEndPoints[1] = p2 + lineSegment2Normalized * delta;

		float t_d = 2 * delta / feedrate;

		FPoint3 c3 = splineEndPoints[0];
		FPoint3 c2 = v1_vect;
		FPoint3 c1 = (v2_vect - c2) / (2 * t_d);

		FMatrix3x3 splineMatrix = FMatrix3x3();
		splineMatrix.m[0][0] = c1.x;
		splineMatrix.m[1][0] = c1.y;
		splineMatrix.m[2][0] = c1.z;
		splineMatrix.m[0][1] = c2.x;
		splineMatrix.m[1][1] = c2.y;
		splineMatrix.m[2][1] = c2.z;
		splineMatrix.m[0][2] = c3.x;
		splineMatrix.m[1][2] = c3.y;
		splineMatrix.m[2][2] = c3.z;

		float halfTimeC1 = pow(t_d / 2, 2);
		float halfTimeC2 = t_d / 2;
		float halfTimeC3 = 1;

		FPoint3 splineHalfwayPoint = FPoint3(
			halfTimeC1 * splineMatrix.m[0][0] + halfTimeC2 * splineMatrix.m[0][1] + halfTimeC3 * splineMatrix.m[0][2],
			halfTimeC1 * splineMatrix.m[1][0] + halfTimeC2 * splineMatrix.m[1][1] + halfTimeC3 * splineMatrix.m[1][2],
			halfTimeC1 * splineMatrix.m[2][0] + halfTimeC2 * splineMatrix.m[2][1] + halfTimeC3 * splineMatrix.m[2][2]
		);

		float chord_error = (p2 - splineHalfwayPoint).vSize();

		logAlways("chord error = %f\n" \
				  "delta = %f\n" \
				  "acceleration vector = <%f, %f, %f>\n", chord_error, delta, c1.x / 2, c1.y / 2, c1.z / 2);
		
		float ratio = ceil(t_d / (1.0 / CONTROL_LOOP_FREQ));
		delta = (1.0 / CONTROL_LOOP_FREQ) * feedrate * ratio / 2;
		logAlways("new delta = %f\n", delta);
	}
}

float PathSmoother::computeFeedrate(FPoint3 s1, FPoint3 s2, FPoint3 accelProfile, float delta) {
	float s1size = s1.vSize();
	float s2size = s2.vSize();

	logAlways("[INFO] s1 = <%f, %f, %f>, s2 = <%f, %f, %f>\n", s1.x, s1.y, s1.z, s2.x, s2.y, s2.z);
	logAlways("[INFO] s1size: %f, s2size: %f, delta: %f\n", s1size, s2size, delta);
	logAlways("[INFO] accelProfile: <%f, %f, %f>\n", accelProfile.x, accelProfile.y, accelProfile.z);

	float feedrate_x = accelProfile.x * delta * s1size * s2size / (s2size * s1.x - s1size * s2.x);
	float feedrate_y = accelProfile.y * delta * s1size * s2size / (s2size * s1.y - s1size * s2.y);
	float feedrate_z = accelProfile.z * delta * s1size * s2size / (s2size * s1.z - s1size * s2.z);

	logAlways("[INFO] feedrates = <%f, %f, %f>\n", feedrate_x, feedrate_y, feedrate_z);

	float min_feedrate = feedrate_x;
	if (feedrate_y < min_feedrate) min_feedrate = feedrate_y;
	if (feedrate_z < min_feedrate) min_feedrate = feedrate_z;

	return 2 * sqrt(min_feedrate);
}

float PathSmoother::findMinFeedrate(std::vector<FPoint3>& points, unsigned int start_idx, unsigned int end_idx, FPoint3 accelProfile) {
	// If the indices are the same, no distance
	if (start_idx == end_idx) return 0.0;
	if (start_idx - end_idx == 1) return MAX_FEEDRATE;

	// Define our distance variable and the index to the shortest segment
	float dist2;
	unsigned int idx = 0;

	// Find the shortest segment between start_idx and end_idx
	for (unsigned int point_idx = start_idx; point_idx < end_idx; ++point_idx) {
		float dist2_temp = (points[point_idx] - points[point_idx + 1]).vSize2();

		if (point_idx == 0 || dist2_temp < dist2) {
			dist2 = dist2_temp;
			idx = point_idx;
		}
	}

	logAlways("[INFO] Smallest distance^2: %f\n", dist2);

	// The smallest feedrate may be between the shortest segment and the segment before it,
	// or the shortest segment and the segment after it
	float delta = sqrt(dist2) * 0.5;
	float f1 = computeFeedrate(points[idx] - points[idx - 1], points[idx + 1] - points[idx], accelProfile, delta);
	float f2 = computeFeedrate(points[idx + 1] - points[idx], points[idx + 2] - points[idx + 1], accelProfile, delta);

	if (f1 < f2) return f1;
	else return f2;
}

FPoint3 PathSmoother::pointFromGCommand(std::shared_ptr<GCommand> comm) {
	FPoint3 point;
	switch (comm->getType()) {
		// G0 command, point is specified XYZ point
		case 0: {
			std::shared_ptr<G0> g0comm = std::static_pointer_cast<G0>(comm);
			
			point = FPoint3(g0comm->X(), g0comm->Y(), g0comm->Z());
			break;
		}
		// G1 command, point is specified XYZ point
		case 1: {
			std::shared_ptr<G1> g1comm = std::static_pointer_cast<G1>(comm);
			
			point = FPoint3(g1comm->X(), g1comm->Y(), g1comm->Z());
			break;
		}
		// G28 command, point is the specified HOME values
		case 28: {
			std::shared_ptr<G28> g28comm = std::static_pointer_cast<G28>(comm);

			point = FPoint3(X_HOME, Y_HOME, Z_HOME);
			break;
		}

		default:
			logAlways("[ERROR] Could not identify GCode command with type %d\n", comm->getType());
			point = FPoint3(0, 0, 0);
	}

	return point;
}

void PathSmoother::processGCommand(std::string command) {
	// Split the command on spaces
	std::vector<std::string> tokens = split(command, ' ');

	// Get the GCode command number
	int commandNr = std::stoi(tokens[0].substr(1));

	// Handle the various GCode command types
	switch (commandNr) {
		// G0 & G1
		case 0:
		case 1:
			// Error check
			if (tokens.size() <= 1) return;

			// Indicates which parameters were provided
			bool hadX, hadY, hadZ;

			// Stores the parameter values
			double x, y, z;

			// Assume no parameters given
			hadX = hadY = hadZ = false;

			// Set the parameter values to the last values, we'll update these next
			x = lastX;
			y = lastY;
			z = lastZ;

			// Iterate through each parameter
			for (unsigned int token_idx = 1; token_idx < tokens.size(); ++token_idx) {
				// Retrieve the parameter
				std::string token = tokens[token_idx];

				// Check to see the parameter type and update accordingly
				char param = token[0];
				if (param == 'X') {
					hadX = true;
					x = std::stod(token.substr(1));
				}
				else if (param == 'Y') {
					hadY = true;
					y = std::stod(token.substr(1));
				}
				else if (param == 'Z') {
					hadZ = true;
					z = std::stod(token.substr(1));
				}
			}

			// If the G-command updated either the x, y, or z-position, add it in
			if (hadX || hadY || hadZ) {
				if (commandNr == 0) {
					auto command = std::make_shared<G0>(x, y, z);
					layerCommands.push_back(command);
				} else {
					auto command = std::make_shared<G1>(x, y, z);
					layerCommands.push_back(command);
				}

				layerPoints.push_back(FPoint3(x, y, z));

				// Update the last x, y, and z values to be the current values
				lastX = x;
				lastY = y;
				lastZ = z;
			}
			break;

		// G28
		case 28: {
			// Indicates which axes should be homed
			bool x, y, z;

			// If no axes specified, home all axes
			if (tokens.size() == 1) {
				x = y = z = true;
			}
			// Otherwise determine which were specified
			else {
				x = y = z = false;

				for (unsigned int idx = 0; idx < tokens.size(); ++idx) {
					std::string token = tokens[idx];

					if (token.compare("X") == 0) {
						x = true;
					}
					else if (token.compare("Y") == 0) {
						y = true;
					}
					else if (token.compare("Z") == 0) {
						z = true;
					}
				}
			}

			// Add the command to the list
			auto command = std::make_shared<G28>(x, y, z);
			layerCommands.push_back(command);
			layerPoints.push_back(FPoint3(x, y, z));
			break;
		}

		// Everything else
		default:
			break;
	}
}

}