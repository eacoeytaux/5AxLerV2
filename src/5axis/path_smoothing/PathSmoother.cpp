#include "PathSmoother.hpp"
#include "../../utils/logoutput.h"
#include <iostream>
#include <fstream>
#include <string.h>

namespace cura {

PathSmoother::PathSmoother(char* gcodeFilePath) {
	// Set the initial value of x/y/z point to be 0, 0, 0
	lastX = lastY = lastZ = 0;

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
	// Iterate through each point on the path
	for (unsigned int command_idx = 0; command_idx < layerCommands.size(); ++ command_idx) {
		std::shared_ptr<GCommand> comm = layerCommands[command_idx];

		switch (comm->getType()) {
			case 0: {
				std::shared_ptr<G0> g0comm = std::static_pointer_cast<G0>(comm);

				logAlways("(%d) %d: %f, %f, %f\n", command_idx, g0comm->getType(), g0comm->X(), g0comm->Y(), g0comm->Z());
				break;
			}

			case 1: {
				std::shared_ptr<G1> g1comm = std::static_pointer_cast<G1>(comm);

				logAlways("(%d) %d: %f, %f, %f\n", command_idx, g1comm->getType(), g1comm->X(), g1comm->Y(), g1comm->Z());
				break;
			}
		}
	}
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
			break;
		}

		// Everything else
		default:
			break;
	}
}

}