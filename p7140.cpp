#include "p7140.h"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <iostream>
#include <stdio.h>

using namespace Pentek;

////////////////////////////////////////////////////////////////////////////////////////
p7140::p7140(std::string devName):
p71xx(devName) {
}

////////////////////////////////////////////////////////////////////////////////////////
p7140::~p7140() {
}

////////////////////////////////////////////////////////////////////////////////////////
p7140dn::p7140dn(std::string devName, std::string dnName, int decrate):
p7140(devName),
_dnName(dnName),
_decrate(decrate),
_dnFd(-1)
{
	// verify that the card was found
	if (!ok())
		return;

	// create the down convertor name
	_dnName = devName + "/dn/" + _dnName;

	 // open it
	_dnFd = open(_dnName.c_str(), O_RDONLY);
	if (_dnFd < 0) {
		_ok = false;
		return;
	}

	// set the clock source
	int clockSource;

	//  clockSource = CLK_SRC_FRTPAN;
	clockSource = CLK_SRC_INTERN;

	if (ioctl(_dnFd, FIOCLKSRCSET, clockSource) == -1)
	{
		std::cerr << "unable to set the clock source for "
		<< _dnName << std::endl;
		perror("");
		_ok = false;
		return;
	}

	// set the clock sample rate
	double doublearg = 100.0e6;
	if (ioctl(_dnFd, FIOSAMPRATESET, &doublearg) == -1) {
		std::cerr << "unable to set the clock rate for "
		<< _dnName << std::endl;
		_ok = false;
		return;
	}

	// set the decimation rate
	if (ioctl(_dnFd, FIODECIMSET, _decrate) == -1) {
		std::cerr << "unable to set the decimation rate for "
			  << _dnName << " to " << _decrate << std::endl;
		perror("");
		_ok = false;
		return;
	}
	std::cout << "decimation set to " << _decrate << std::endl;

	// flush the device read buffers
	if (ioctl(_dnFd, FIOFLUSH, 0) == -1)
	{
		std::cerr << "unable to flush for "
		<< _dnName << std::endl;
		perror("");
		_ok = false;
		return;
	}

	// clear the over/under run counters
	if (overUnderCount() < 0) {
		_ok = false;
		return;
	}

}

////////////////////////////////////////////////////////////////////////////////////////
p7140dn::~p7140dn() {
	if (_dnFd >=0)
		close (_dnFd);
}


///////////////////////////////////////////////////////////
int
p7140dn::overUnderCount() {

	if (!_ok)
		return -1;

  int count = ioctl(_dnFd, FIOGETOVRCNT);
  if (count == -1)
  {
    std::cerr << "unable to get ovr/under for "
  	<< _dnName << std::endl;
    perror("");
    _ok = false;
    return -1;
  }

  // clear the overrun counter
  if (ioctl(_dnFd, FIOCLROVRCNT) == -1)
  {
    std::cerr << "unable to clear ovr/under for "
  	<< _dnName << std::endl;
    perror("");
    _ok = false;
    return -1;
  }

  return count;
}

////////////////////////////////////////////////////////////////////////////////////////
int
p7140dn::read(char* buf, int bufsize) {

    if (!_ok)
       return -1;

	int n = ::read(_dnFd, buf, bufsize);

	if (n < 0)
		_ok = false;

	return n;

}

////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////
