/**
 * SerialComms accepts a serial port and provides an abstraction layer for performing
 * serial communications on Linux and OSX
 *
 * Created by Alexandre Pauwels on 11/4/2016
 */

#ifndef SERIAL_COMMS
#define SERIAL_COMMS

#include <string>

namespace cura {

class SerialComms {
public:
	SerialComms(std::string comPort);

private:
	std::string m_COMPort;
	int m_USBFD;
};

}

#endif
