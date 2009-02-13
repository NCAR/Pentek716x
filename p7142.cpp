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
  _dnFd = open(_dnName.c_str(), O_RDONLY);
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
  double sampleClockHz, double ncoFreqHz, bool simulate):
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
  _upName = devName + "/up/" + _upName;
  _mem2Name = devName + "/mem2";

  // open upconverter
  _upFd = open(_upName.c_str(), O_RDONLY);
  if (_upFd < 0) {
    std::cerr << "unable to open " << _upName << std::endl;
    _ok = false;
    return;
  }

  dumpDACregs(_upFd);

  int clockSource;
  clockSource = CLK_SRC_FRTPAN;
  //	clockSource = CLK_SRC_INTERN;
  // set the clock source
  doIoctl(_upFd, FIOCLKSRCSET, clockSource, "unable to set the DAC clock source");
 
  // sample rate
  doIoctl(_upFd, FIOSAMPRATESET, sampleClockHz, "unable to set the DAC sample rate");
 
  // PLLVDD disable/enable
  doIoctl(_upFd, PLLVDDSET, 1, "ioctl DAC PLLVDDSET failed", false);

  // NCO frequency
  doIoctl(_upFd , FIONCOSET, ncoFreqHz, "unable to set the DAC tuning frequency");
  
  dumpDACregs(_upFd);

  _ok = true;
}

////////////////////////////////////////////////////////////////////////////////////////
p7142up::~p7142up() {
  if (_upFd >=0)
    close (_upFd);
}


////////////////////////////////////////////////////////////////////////////////////////
void
p7142up::dumpDACregs(int fd) {
	for (int i = 0; i < 32; i++) {
		// get value
		unsigned short val = getDACreg(fd, i);
		// print hex
		std::cout << "DAC register " << i;
		std::cout << std::ios::hex;
		std::cout << val << std::endl;
		std::cout << std::ios::dec;
		// print binary
		unsigned char mask = 0x8f;
		for (int i = 0; i < 8; i++) {
			std::cout << " ";
			std::cout << ((val & mask)? "1":"0");
			mask /= 2;
		}
		std::cout << std::endl;
	}
}


////////////////////////////////////////////////////////////////////////////////////////
unsigned char
p7142up::getDACreg(int fd, int reg) {
	
  ARG_PEEKPOKE pp;

  pp.offset = reg;
  pp.page = 0;
  pp.mask = 0;

  ioctl(fd,FIOREGGET,(long)&pp);
  
  return(pp.value); 
}

////////////////////////////////////////////////////////////////////////////////////////
void
p7142up::write(short* data, int n) {

  int mem2depth = n/2;// memory depth in 4 byte words
 
  int memFd = open(_mem2Name.c_str(), O_RDWR, 0);
  if (memFd < 0) {
    std::cerr << "cannot access " << _mem2Name << "\n";
    perror("");
    exit(1);
  }
 
  // set the memory bank depth
  doIoctl(memFd, FIODEPTHSET, mem2depth, "unable to set the memory depth");

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
  doIoctl(_upFd, MEMROUTESET, 1, "ioctl MEMROUTESET for DAC failed");
  
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
