#ifndef PATH_SMOOTHER_H
#define PATH_SMOOTHER_H

#include <vector>
#include <string>
#include <sstream>
#include <memory>

namespace cura {

class GCommand;

class PathSmoother {
private:
	// Since G-commands often omit a parameter (e.g. z is specified once and then unspecified
	// for all G-commands after which are at the same z), this keeps track of the most updated
	// value for each <x, y, z> so that every G0/G1 object can have all three values explicitly
	// stated regardless of whether they were specified in the g-code
	double lastX, lastY, lastZ;

	// Stores all G-commands for a single layer
	std::vector<std::shared_ptr<GCommand>> layerCommands;

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
	double x, y, z;

public:
	G0(double _x, double _y, double _z) : GCommand(0), x(_x), y(_y), z(_z) {};
	double X() { return x; }
	double Y() { return y; }
	double Z() { return z; }
};

class G1 : public GCommand {
private:
	double x, y, z;

public:
	G1(double _x, double _y, double _z) : GCommand(1), x(_x), y(_y), z(_z) {};
	double X() { return x; }
	double Y() { return y; }
	double Z() { return z; }
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