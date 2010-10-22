#include "p71xx.h"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <iostream>
#include <cstdio>
#include <cstdlib>


using namespace Pentek;

////////////////////////////////////////////////////////////////////////////////////////
p71xx::p71xx(std::string devName, bool simulate):
_ok(false),
_devName(devName),
_ctrlFd(-1),
_simulate(simulate)
{
    // If we're simulating, things are simple...
	if (_simulate) {
		_ok = true;
		return;
	}

	if (_devName.size() < 1)
		return;

	// remove trailing slash
	if (_devName[_devName.size()-1] == '/') {
		_devName.erase(_devName.size()-1,1);
	}

	// If we're not simulating, open the control device
	_devCtrl = _devName + "/ctrl";
    _ctrlFd = open(_devCtrl.c_str(), O_RDWR);
    if (_ctrlFd < 0) {
        std::cerr << "Unable to open p71xx control device!" << std::endl;
        return;
    }

	_ok = true;
}

////////////////////////////////////////////////////////////////////////////////////////
p71xx::~p71xx() {
    if (_ctrlFd >= 0)
        close(_ctrlFd);
}

////////////////////////////////////////////////////////////////////////////////////////
bool
p71xx::ok() const {
	return _ok;
}

/////////////////////////////////////////////////////////////////////
double
p71xx::gauss(double mean, double stdDev) {

	// create a normally distributed random number,
	// using this nifty little algorithm.

	double x = rand()/(1.0*RAND_MAX);
	double y = rand()/(1.0*RAND_MAX);
	double u = sqrt(-2.0*log10(x))*cos(2.0*M_PI*y);

	// set the mean std deviation
	return stdDev * u + mean;
}

//////////////////////////////////////////////////////////////////////
int
p71xx::doIoctl(int fd, int ioctlCode, void* arg, std::string errMsg, bool doexit) {

   int status = ioctl(fd, ioctlCode, arg);
   if (status == -1) {
    std::cout << errMsg << std::endl;
    perror("");
    if (doexit)
    exit(1);
   }

   return status;
}

//////////////////////////////////////////////////////////////////////
int
p71xx::doIoctl(int fd, int ioctlCode, int arg, std::string errMsg, bool doexit) {
   return doIoctl(fd, ioctlCode, (void*) arg, errMsg, doexit);
}

//////////////////////////////////////////////////////////////////////
int
p71xx::doIoctl(int fd, int ioctlCode, double arg, std::string errMsg, bool doexit) {

   double doubleArg = arg;
   return doIoctl(fd, ioctlCode, (void*) &doubleArg, errMsg, doexit);
}

////////////////////////////////////////////////////////////////////////////////////////
int
p71xx::bufset(int fd, int intbufsize, int bufN) {

	BUFFER_CFG bc;
	bc.bufno = 0;
	bc.bufsize = bufN*intbufsize;
	bc.intbufsize = intbufsize;
	bc.physAddr = 0;

	int status = ioctl(fd, BUFSET, &bc);
	if (status == -1) {
		std::cout << "Error setting pentek buffer sizes" << std::endl;
		perror("");
	}
	return status;
}
