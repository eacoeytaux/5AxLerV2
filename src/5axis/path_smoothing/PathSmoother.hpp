#ifndef PATH_SMOOTHER_H
#define PATH_SMOOTHER_H

#include <vector>
#include <string>
#include <sstream>
#include <memory>
#include "../../utils/floatpoint.h"

namespace cura {

#define MAX_ACCEL 5				// mm/s^2
#define MAX_FEEDRATE 10			// mm/s
#define X_HOME 0				// mm
#define Y_HOME 0				// mm
#define Z_HOME 203				// mm
#define CONTROL_LOOP_FREQ 20000	// Hz
#define MAX_ANGLE 0.17453292519943295769236907684886 // 10 degrees

class GCommand;

class PathSmoother {
private:
	// Since G-commands often omit parameters (e.g. z is specified once and then unspecified
	// for all G-commands after which are at the same z), this keeps track of the most updated
	// value for each <x, y, z> so that every G0/G1 object can have all three values explicitly
	// stated regardless of whether they were specified in the g-code
	float lastX, lastY, lastZ;

	// Stores the current position of the print-head
	float currX, currY, currZ;

	// Stores all G-commands for a single layer
	std::vector<std::shared_ptr<GCommand>> layerCommands;
	std::vector<FPoint3> layerPoints;

	/**
	 * Takes in a string containing a G-command, processes it into a GCommand object,
	 * and then adds the object to the layerCommands vector.
	 *
	 * @param command The command string
	 */
	void processGCommand(std::string command);

	/**
	 * Runs the path-smoothing algorithm on all points inside layerCommands
	 */
	void processLayer();

	/**
	 * Splits the given string according to the provided delimiter and
	 * stores the resulting tokens in result
	 *
	 * @param s The string to split
	 * @param delim A character to act as a delimiter
	 * @param result Where to store the resulting tokens
	 */
	template<typename Out>
	void split(const std::string &s, char delim, Out result) {
		std::stringstream ss;
		ss.str(s);
		std::string item;
		while (std::getline(ss, item, delim)) {
			if (!item.empty()) {
				*(result++) = item;
			}
		}
	}

	/**
	 * Splits the given string according to the provided delimiter and
	 * returns a vector of string tokens containing the result
	 *
	 * @param s The string to split
	 * @param delim A character to act as a delimiter
	 *
	 * @return A vector of string tokens
	 */
	std::vector<std::string> split(const std::string &s, char delim) {
		std::vector<std::string> elems;
		split(s, delim, std::back_inserter(elems));
		return elems;
	}

	/**
	 * Takes in a GCommand and returns the command's
	 * x/y/z values in an FPoint3. If the command is not recognized, the FPoint3
	 * has value <0, 0, 0> TODO: There's a better way to handle invalid G-commands
	 *
	 * @param comm A pointer to the GCommand
	 *
	 * @return The FPoint3 containing the G-command positional values
	 */
	FPoint3 pointFromGCommand(std::shared_ptr<GCommand> comm);

	/**
	 * Takes in an array of 3D points and two indices in the array, and finds the lowest
	 * feedrate for all segments between the two indices.
	 *
	 * @param comms The array of points
	 * @param start_idx The index of the first point
	 * @param end_idx The index of the last point (inclusive)
	 *
	 * @return The minimum feedrate
	 */
	float findMinFeedrate(std::vector<FPoint3>& points, unsigned int, unsigned int, FPoint3);

	/**
	 * Computes the feedrate for a spline between two line segments given an acceleration
	 * profile specifying the maximum acceleration in each axis and a delta specifying
	 * the maximum amount of distance along a line segment the spline has
	 *
	 * @param segment1 The first line segment
	 * @param segment2 The second line segment
	 * @param accelProfile A 3D vector specifying maximum acceleration in all 3 axes
	 * @param delta The spline delta
	 *
	 * @return The feedrate as a float
	 */
	float computeFeedrate(FPoint3, FPoint3, FPoint3, float);

	/**
	 * Creates a continuous spline connecting the three given points (representing
	 * two line segments) into a smooth path.
	 *
	 * @param p1 The startpoint of the first line segment
	 * @param p2 The middle point (endpoint of the first line segment, startpoint of the second)
	 * @param p3 The endpoint of the second line segment
	 * @param accelProfile The maximum acceleration on each axis
	 * @param feedrate The feedrate for the spline
	 */
	void createSpline(FPoint3 p1, FPoint3 p2, FPoint3 p3, FPoint3 accelProfile, float feedrate);
public:
	/**
	 * PathSmoother accepts a string giving the path to the GCode
	 * file and processes it into a velocity profile file
	 *
	 * @param gcodeFilePath Path to the GCode file
	 */
	PathSmoother(char* gcodeFilePath);
};

class GCommand {
private:
	unsigned int type;

public:
	GCommand(unsigned int _type) : type(_type) {};
	unsigned int getType() { return type; }
};

class G0 : public GCommand {
private:
	float x, y, z;

public:
	G0(float _x, float _y, float _z) : GCommand(0), x(_x), y(_y), z(_z) {};
	float X() { return x; }
	float Y() { return y; }
	float Z() { return z; }
};

class G1 : public GCommand {
private:
	float x, y, z;

public:
	G1(float _x, float _y, float _z) : GCommand(1), x(_x), y(_y), z(_z) {};
	float X() { return x; }
	float Y() { return y; }
	float Z() { return z; }
};

class G28 : public GCommand {
private:
	bool x, y, z;

public:
	G28(bool _x, bool _y, bool _z) : GCommand(28), x(_x), y(_y), z(_z) {};
	bool X() { return x; }
	bool Y() { return y; }
	bool Z() { return z; }
};

}

#endif