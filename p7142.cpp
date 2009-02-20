#include "p7142.h"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <iostream>
#include <stdio.h>


using namespace Pentek;

////////////////////////////////////////////////////////////////////////////////////////
p7142::p7142(std::string devName, bool simulate):
  p71xx(devName, simulate)
{
}

////////////////////////////////////////////////////////////////////////////////////////
p7142::~p7142() {
}

////////////////////////////////////////////////////////////////////////////////////////
p7142dn::p7142dn(std::string devName, std::string dnName, int bypdiv,
		 bool simulate, int simPauseMS):
  p7142(devName, simulate),
  _dnName(dnName),
  _bypdiv(bypdiv),
  _dnFd(-1),
  _simPauseMS(simPauseMS)
{
  // verify that the card was found
  if (!ok()) {
    std::cerr << "p7142 card not ready" << std::endl;
    return;
  }

  if (_simulate)
    return;

  // create the down convertor name
  _dnName = devName + "/dn/" + _dnName;

  // open it
  _dnFd = open(_dnName.c_str(), O_RDWR);
  if (_dnFd < 0) {
    std::cerr << "unable to open " << _dnName << std::endl;
    _ok = false;
    return;
  }

  // set the clock source
  int clockSource;

  clockSource = CLK_SRC_FRTPAN;
  //	clockSource = CLK_SRC_INTERN;

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
    perror("");
    _ok = false;
    return;
  }

  // set the decimation rate
  if (ioctl(_dnFd, FIOBYPDIVSET, _bypdiv) == -1) {
    std::cerr << "unable to set the bypass decimation rate for "
	      << _dnName << " to " << _bypdiv << std::endl;
    perror("");
    _ok = false;
    return;
  }
  std::cout << "bypass decimation set to " << _bypdiv << std::endl;

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
  if (overUnderCount() < 0)
    return;

  _ok = true;
}

////////////////////////////////////////////////////////////////////////////////////////
p7142dn::~p7142dn() {
  if (_dnFd >=0)
    close (_dnFd);
}

///////////////////////////////////////////////////////////
int
p7142dn::overUnderCount() {

  if (!_ok)
    return -1;

  // if simulate, indicate no errors.
  if (_simulate) {
    return 0;
  }

  int count = ioctl(_dnFd, FIOGETOVRCNT);
  if (count == -1)
    {
      std::cout << "unable to get ovr/under for "
		<< _dnName << std::endl;
      perror("");
      _ok = false;
      return -1;
    }

  // clear the overrun counter
  if (ioctl(_dnFd, FIOCLROVRCNT) == -1)
    {
      std::cout << "unable to clear ovr/under for "
		<< _dnName << std::endl;
      perror("");
      _ok = false;
      return -1;
    }

  return count;
}

////////////////////////////////////////////////////////////////////////////////////////
int
p7142dn::read(char* buf, int bufsize) {

  if (!_ok)
    return -1;

  // if in simulation mode, create some random values
  // and return a full buffer.
  if (_simulate) {
    short* sbuf = (short*)buf;
    int wl = 200;
    double fact = 1.0 + 0.05*(1.0*rand())/RAND_MAX;
    for (int i = 0; i < bufsize/2; i = i + 2) {
      sbuf[i] = (10000.0 * sin(2.0*M_PI*i/wl)*fact);
      sbuf[i+1] = (10000.0 * cos(2.0*M_PI*i/wl)*fact);
    }
    usleep(_simPauseMS*1000);
    return bufsize;
  }

  int n = ::read(_dnFd, buf, bufsize);

  if (n < 0)
    _ok = false;

  return n;
}

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

p7142up::p7142up(std::string devName, std::string upName,
  double sampleClockHz, double ncoFreqHz, long interp, bool simulate):
  p7142(devName, simulate),
  _upName(upName),
  _mem2Name(""),
  _upFd(-1)
{

  // verify that the card was found
  if (!ok()) {
    std::cerr << "p7142 card not ready" << std::endl;
    return;
  }

  if (_simulate)
    return;

  // create the up device names
  _upName = _devName + "/up/" + _upName;
  _mem2Name = _devName + "/mem2";

  // open upconverter
  _upFd = open(_upName.c_str(), O_RDONLY);
  if (_upFd < 0) {
    std::cerr << "unable to open " << _upName << std::endl;
    _ok = false;
    return;
  }

  /// @todo Disable the register dump via the REGGET ioctl; for some strange reason
  /// if we do a FIOSAMPRATESET, then all of the later REGGETs return 0xff.
  /// However, the driver parameters are getting set correctly. I think there
  /// is something broken in the REGGET ioctl.
  std::cout << "DAC registers after opening " << _upName << std::endl;
  dumpDACregs(_upFd);

  long clockSource;
  clockSource = CLK_SRC_FRTPAN;
  //	clockSource = CLK_SRC_INTERN;

  // set the clock source
  ioctl(_upFd, FIOCLKSRCSET, clockSource);

  // sample rate
  /// @todo commented out due to bug in v2.3 of driver. Enable when
  /// steve Rotinger (Pentek) provides an updated driver. In the meantime,
  /// the sample clock can be set via the p7140_clkbrate=125000000 parameter
  /// during the drive load (via modprobe)
  //  ioctl(_upFd, FIOSAMPRATESET, &sampleClockHz);

  // interpolation
  ioctl(_upFd, INTERPSET, interp);

  // NCO frequency
  ioctl(_upFd , FIONCOSET, &ncoFreqHz);

  // Disable the register dump, for the reasons given above
  std::cout << "DAC registers after configuration " << _upName << std::endl;
  dumpDACregs(_upFd);

  // close the upconverter, otherwise we won't be able to acces the mem2 device
  close(_upFd);
  _upFd = -1;

  _ok = true;
}

////////////////////////////////////////////////////////////////////////////////////////
p7142up::~p7142up() {
  if (_upFd >=0) {
	  std::cout << __FUNCTION__ << " closing upconverter" << std::endl;
      close (_upFd);
  }
}


////////////////////////////////////////////////////////////////////////////////////////
void
p7142up::dumpDACregs(int fd) {
	for (int i = 0; i < 32; i++) {
		// get value
		unsigned short val = getDACreg(fd, i);
		std::cout << "DAC register "  << std::hex  << i << std::dec << ": ";
		// print binary
		unsigned char mask = 0x8f;
		for (int i = 0; i < 8; i++) {
			std::cout << " ";
			std::cout << ((val & mask)? "1":"0");
			mask /= 2;
		}
		// print hex
		std::cout << "  " << std::hex << val << std::dec << "     ";
		std::cout << std::endl;
	}
}

////////////////////////////////////////////////////////////////////////////////////////
unsigned char
p7142up::getDACreg(int fd, int reg) {

  ARG_PEEKPOKE pp;
  pp.offset = reg;
  pp.page = 2;
  pp.mask = 0;

  int status = ioctl(fd,FIOREGGET,(long)&pp);
  if (status < 0) {
	  perror("FIOREGGET ioctl error");
  }

  return(pp.value);
}

////////////////////////////////////////////////////////////////////////////////////////
void
p7142up::setDACreg(int fd, int reg, unsigned short val) {

  ARG_PEEKPOKE pp;
  pp.offset = reg;
  pp.page = 2;
  pp.mask = 0;
  pp.value = val;

  int status = ioctl(fd,FIOREGSET,(long)&pp);
  if (status < 0) {
	  perror("FIOREGSET ioctl error");
  }
}

////////////////////////////////////////////////////////////////////////////////////////
void
p7142up::write(short* data, int n) {

  long mem2depth = n/2;// memory depth in 4 byte words
  int memFd = open(_mem2Name.c_str(), O_WRONLY);
  if (memFd < 0) {
    std::cerr << "cannot access " << _mem2Name << "\n";
    perror("");
    exit(1);
  }

  // set the memory bank depth
  ioctl(memFd, FIODEPTHSET, mem2depth);

  // It appears that you need to do the
  // following lseek to insure writing to
  // the start of memory.
  lseek(memFd, 0, SEEK_SET);

  // write the baseband to memory bank 2
  if (::write(memFd, (char*)(data), mem2depth*4)
      != mem2depth*4) {
    std::cerr << "unable to fill pentek memory bank 2" << std::endl;
    perror("");
    exit(1);
  }

  close (memFd);

}


////////////////////////////////////////////////////////////////////////////////////////
void
p7142up::startDAC() {
  // this seems to be the way that you start it; by
  // selecting the memory route

  _upFd = open(_upName.c_str(), O_RDWR);
  long route = 1;
  ioctl(_upFd, MEMROUTESET, route);
  close(_upFd);
  _upFd = -1;

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
