#include "SerialComms.hpp"
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include "../../utils/logoutput.h"
#include <cstring>

namespace cura {

SerialComms::SerialComms(std::string COMPort) {
	m_COMPort = COMPort;

	m_USBFD = open(COMPort.c_str(), O_RDWR | O_NONBLOCK | O_NDELAY);
	if (m_USBFD < 0) {
		log("[ERROR] Could not open communications to the printer on %s: %s\n", m_COMPort.c_str(), strerror(errno));
	} else {
		log("[INFO] Connected to serial device: %s\n", m_COMPort.c_str());
	}

	int closeSuccess = close(m_USBFD);
	if (closeSuccess < 0) {
		log("[ERROR] Could not close communications to the printer on %s: %s\n", m_COMPort.c_str(), strerror(errno));
	} else {
		log("[INFO] Closed serial device: %s\n", m_COMPort.c_str());
	}
}

}