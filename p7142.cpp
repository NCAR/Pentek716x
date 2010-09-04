#include "p7142.h"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <cerrno>
#include <cstring>  // for memcpy

using namespace Pentek;
int drv_peekL(int fd,unsigned intaddr);
int drv_pokeL(int fd,unsigned int addr,unsigned int value);

////////////////////////////////////////////////////////////////////////////////////////
p7142::p7142(std::string devName, bool simulate):
  p71xx(devName, simulate)
{
}

////////////////////////////////////////////////////////////////////////////////////////
p7142::~p7142() {
}

////////////////////////////////////////////////////////////////////////////////////////
p7142dn::p7142dn(
		std::string devName,
		int chanId,
		int decimation,
		bool simulate,
		int simWaveLength,
	    bool sim4bytes,
		bool internalClock):
  p7142(devName, simulate),
  _chanId(chanId),
  _dnFd(-1),
  _simWaveLength(simWaveLength),
  _angleCount(0),
  _sim4bytes(sim4bytes)
{
  // create the read mutex
  //_readMutex.create();

  // verify that the card was found
  if (!ok()) {
    std::cerr << "p7142 card not ready" << std::endl;
    return;
  }

  if (_simulate) {
  	_dnName = "dnSimulate";
  	_ok = true;
    return;
  }

  char c[2];
  c[0] = '0' + _chanId;
  c[1] = 0;
  std::string dnchan(c);
  dnchan += "BR";

  // create the down converter name
  _dnName = devName + "/dn/" + dnchan;

  // open it
  _dnFd = open(_dnName.c_str(), O_RDWR);
  if (_dnFd < 0) {
    std::cerr << "unable to open " << _dnName << std::endl;
    perror("error opening pentek device:");
    _ok = false;
    return;
  }

  // set the clock source
  int clockSource;

  if (internalClock) {
	  clockSource = CLK_SRC_INTERN;
  } else {
	  clockSource = CLK_SRC_FRTPAN;
  }

  if (ioctl(_dnFd, FIOCLKSRCSET, clockSource) == -1)
    {
      std::cerr << "unable to set the clock source for "
		<< _dnName << std::endl;
      perror("");
      _ok = false;
      return;
    }

  // set the bypass divider
  if (! setBypassDivider(decimation)) {
      _ok = false;
      return;
  }

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

////////////////////////////////////////////////////////////////////////////////////////
std::string
p7142dn::dnName() {
	return _dnName;
}

////////////////////////////////////////////////////////////////////////////////////////
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
  if (!_simulate) {
	  // not in simulation mode; do a proper read from the device
	  int n;
	  n = ::read(_dnFd, buf, bufsize);

	  if (n > 0)
		  _bytesRead += n;

	  if (n < 0)
		_ok = false;

	  return n;
  }

  // In simulation mode, create some random values
  // and return a full buffer.

  // there is a bug in this code. It assumes that bufsize
  // is an integral number of I/Q pairs. For the time being,
  // detect if this is not true and just abort.
  /// @todo Fix read function to return an arbitrary number of bytes
  /// when in simulation mode
  assert ((bufsize % 4) == 0);

  short* sbuf = (int16_t*)buf;
  int*   ibuf = (int32_t*)buf;

  // 4  or 8 bytes per IQ pair
  int nPairs;
  if (_sim4bytes) {
	  nPairs = (bufsize) / 8;
  } else {
	  nPairs = (bufsize) / 4;
  }
  for (int p = 0; p < nPairs; p++) {
	// noise is +/-10% amplitude
    double noise = 0.1 * ((2.0 * rand()) / RAND_MAX - 1.0);
    // Noisy sine wave, with wavelength of _simWaveLength gates
    // The wavelength varies across the range
    if (_angleCount == _simWaveLength) {
    	_angleCount = 0;
    }

    double angle = ((double)_angleCount++)/ _simWaveLength;
    double I = 10000 * (sin((2 * angle * M_PI)) + noise);
    double Q = 10000 * (cos((2 * angle * M_PI)) + noise);
    if (_sim4bytes) {
		*ibuf++ = (int32_t) I; // I
		*ibuf++ = (int32_t) Q; // Q
    } else {
		*sbuf++ = (int16_t) I; // I
		*sbuf++ = (int16_t) Q; // Q
    }
  }
  _bytesRead += bufsize;

  return bufsize;
}

////////////////////////////////////////////////////////////////////////////////////////
int
p7142dn::fd() {
	return _dnFd;
}

////////////////////////////////////////////////////////////////////////////////////////
void p7142dn::flush() {
	  // flush the device read buffers. This will clear the fifos,
	  // which will probably contain data since we are not able yet
	  // to disable the timers, and so the fifos may have been
	  // filling. When we do implement true timer control, this
	  // flush will probably not be needed.
  if (_simulate)
      return;
  
  if (ioctl(_dnFd, FIOFLUSH, 0) == -1) {
    std::cerr << "unable to flush for " << _dnName << std::endl;
    perror("");
  }

  std::cout << "flush performed on " << _dnName << std::endl;
}

////////////////////////////////////////////////////////////////////////////////////////
bool
p7142dn::usingInternalClock() const {
    if (_simulate)
        return(false);
    
    int clockSource;
    if ((clockSource = ioctl(_dnFd, FIOCLKSRCGET, 0)) == -1) {
        std::cerr << __FUNCTION__ << ": ioctl error on FIOCLKSRCGET: " <<
                strerror(errno);
    }
    return(clockSource == CLK_SRC_INTERN);
}

////////////////////////////////////////////////////////////////////////////////////////
int
p7142dn::bypassDivider() const {
    if (_simulate)
        return(0);
    
    // get the bypass divider
    int bypassdiv;
    if ((bypassdiv = ioctl(_dnFd, FIOBYPDIVGET, 0)) == -1) {
      std::cerr << "Unable to get the bypass divider for "
            << _dnName << std::endl;
      perror(__FUNCTION__);
    }
    return bypassdiv;
}

////////////////////////////////////////////////////////////////////////////////////////
bool
p7142dn::setBypassDivider(int bypassdiv) const {
    if (_simulate)
        return true;
    
    // set the bypass divider
    if (ioctl(_dnFd, FIOBYPDIVSET, bypassdiv) == -1) {
      std::cerr << "unable to set the bypass divider for "
            << _dnName << " to " << bypassdiv << std::endl;
      perror(__FUNCTION__);
      return false;
    }
    return true;
}

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

p7142up::p7142up(std::string devName, std::string upName,
  double sampleClockHz, double ncoFreqHz, char mode, bool simulate):
  p7142(devName, simulate),
  _sampleClockHz(sampleClockHz),
  _ncoFreqHz(ncoFreqHz),
  _interpMode(2),
  _upName(upName),
  _mem2Name(""),
  _upFd(-1),
  _mem2depth(0)
{

  // verify that the card was found
  if (!ok()) {
    std::cerr << "p7142 card not ready" << std::endl;
    return;
  }

  if (_simulate)
    return;
    
	// open Pentek 7142 ctrl device
	_ctrlFd = open(_devCtrl.c_str(), O_RDWR);
	if (_ctrlFd < 0) {
		std::cout << "unable to open Ctrl device\n";
		return;
	}


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

  //std::cout << "DAC registers after opening " << _upName << std::endl;
  //dumpDACregs(_upFd);

  long clockSource;
  clockSource = CLK_SRC_FRTPAN;
  //clockSource = CLK_SRC_INTERN;

  // set the clock source
  ioctl(_upFd, FIOCLKSRCSET, clockSource);

  // sample rate
  /// @todo Sample rate commented out due to bug in v2.3 of driver. Enable when
  /// Steve Rotinger (Pentek) provides an updated driver. In the meantime,
  /// the sample clock can be set via the p7140_clkbrate=125000000 parameter
  /// during the drive load (via modprobe)
  //  ioctl(_upFd, FIOSAMPRATESET, &_sampleClockHz);

  // Set DAC FIFO clock source
  ioctl(_upFd, FIODACCLKSET, 0);

  // Enable PLL on DAC
  ioctl(_upFd, FIOPLLVDDSET, 1);

  // Version: set FIR1 to low pass on DAC ChA and ChB, also disable DAC B, if operating at 48 or 125 MHz
  char version =
	  1 << 7 |				// DAC A sleep
	  0 << 6 |				// DAC B operational
	  0 << 5 |				// hplb, DAC B fir1
	  0 << 4 ;				// hpla, DAC A fir1

	  ;
  setDACreg(_upFd, 0x0, version);

  // Config 0:
  // Bypass DAC FIFOs since we are using PLL,
  // set NCO to high freq,
  // PLL divider = 1,
  // interp = X4L
  char config0;
  if(_sampleClockHz == 125000000)  // Settings for 125 MHz clock
	  config0 =

	  0 << 6 |               // pll_div
	  1 << 5 |               // pll_freq
	  0 << 4 |               // pll_kv
	  _interpMode << 2  |    // interp
	  0 << 1 |               // inv_pllock
	  1 << 0;                // fifo_bypass


  else						 // Settings for 48 MHz Clock
	  config0 =

	  1 << 6 |               // pll_div
	  0 << 5 |               // pll_freq
	  0 << 4 |               // pll_kv
	  _interpMode << 2  |    // interp
	  0 << 1 |               // inv_pllock
	  1 << 0;                // fifo_bypass

  setDACreg(_upFd, 0x01, config0);

  // Config 1: Set input Data two two's complement, non-interleaved
  char config1 = 1 << 4;
  setDACreg(_upFd, 0x02, config1);

  // Config 2: Enable NCO, set cm_mode, enable inv. sync filter
  char config2 = 0x80 | (mode << 1) | 0x1;
  setDACreg(_upFd, 0x03, config2);

  // Config 3: For now just a placeholder
  char config3 = 0x80;
  setDACreg(_upFd, 0x04, config3);

  // Sync Control: Sync NCO, sync coarse mixer, disable FIFO sync
 char sync_cntl = 0x40 | 0x20 | 0x6 << 2;

  setDACreg(_upFd, 0x05, sync_cntl);

  char nco_0;
  char nco_1;
  char nco_2;
  char nco_3;
  ncoConfig(_ncoFreqHz, 4*_sampleClockHz, nco_0, nco_1, nco_2, nco_3);
  std::cout << std::hex <<
    (int)nco_0 << " " <<
    (int)nco_1 << " " <<
    (int)nco_2 << " " <<
    (int)nco_3 << " " <<
    std::dec << std::endl;

  // char nco_freq = 0x0;
  setDACreg(_upFd, 0x09, nco_0); // bits 0-7
  setDACreg(_upFd, 0x0A, nco_1); // bits 8-15
  setDACreg(_upFd, 0x0B, nco_2); // bits 16-23
  //  nco_freq = 0x20;
  setDACreg(_upFd, 0x0C, nco_3); // bits 24-31

  //std::cout << "DAC registers after configuration " << _upName << std::endl;
  //dumpDACregs(_upFd);

  std::cout << "sample clock:     " << sampleClockHz << std::endl;
  std::cout << "nco frequency:    " << ncoFreqHz << std::endl;
  std::cout << "coarse mixer mode:" << (int)mode << std::endl;

  // close the upconverter, otherwise we won't be able to access the mem2 device
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
  
  if (_ctrlFd >= 0) {
  	close(_ctrlFd);
  }
}

////////////////////////////////////////////////////////////////////////////////////////
void
p7142up::dumpDACregs(int fd) {
    if (_simulate) {
        std::cout << "No DAC registers: running in simulation mode." << std::endl;
        return;
    }
    
	for (int i = 0; i < 32; i++) {
		// get value
		char val = getDACreg(fd, i);
		std::cout << "DAC register 0x"  << std::hex  << i << std::dec << ":";
		// print binary
		for (int i = 0; i < 8; i++) {
		    char mask = 1 << (7 - i);
			std::cout << " ";
			std::cout << ((val & mask)? "1":"0");
		}
		// print hex
		std::cout << "  " << std::hex << (((int)val) & 0xff) << std::dec << "     ";
		std::cout << std::endl;
	}
}

////////////////////////////////////////////////////////////////////////////////////////
char
p7142up::getDACreg(int fd, int reg) {
  if (_simulate)
    return 0;

  ARG_PEEKPOKE pp;
  pp.offset = reg;
  pp.page = 0;
  pp.mask = 0;

  int status = ioctl(fd,FIOREGGET,(long)&pp);
  if (status < 0) {
	  perror("FIOREGGET ioctl error");
  }

  return(pp.value);
}

////////////////////////////////////////////////////////////////////////////////////////
void
p7142up::setDACreg(int fd, int reg, char val) {
  if (_simulate)
      return;

  ARG_PEEKPOKE pp;
  pp.offset = reg;
  pp.page = 0;
  pp.mask = 0;
  pp.value = val;

  int status = ioctl(fd,FIOREGSET,(long)&pp);
  if (status < 0) {
	  perror("FIOREGSET ioctl error");
  }
}

////////////////////////////////////////////////////////////////////////////////////////
void
p7142up::write(long* data, int n) {
  if (_simulate)
      return;

  // memory depth in 4 byte words
  _mem2depth = n;
  
  int memFd = open(_mem2Name.c_str(), O_WRONLY);
  if (memFd < 0) {
    std::cerr << "cannot access " << _mem2Name << "\n";
    perror("");
    exit(1);
  }

  // set the memory bank depth
  ioctl(memFd, FIODEPTHSET, _mem2depth);

  // It appears that you need to do the
  // following lseek to insure writing to
  // the start of memory.
  lseek(memFd, 0, SEEK_SET);

  // write the baseband to memory bank 2
  if (::write(memFd, (char*)(data), _mem2depth*4)
    != _mem2depth*4) {
    std::cerr << "unable to fill pentek memory bank 2" << std::endl;
    perror("");
    exit(1);
  }

  close (memFd);

}


////////////////////////////////////////////////////////////////////////////////////////
void
p7142up::startDAC() {
  if (_simulate)
      return;
	
  // close the upconverter so that the memory counter stops running
  if (_upFd != -1) {
  	close(_upFd);
  	_upFd = -1;
  }
  
  _upFd = open(_upName.c_str(), O_RDONLY);
  if (_upFd < 0) {
    std::cerr << "unable to open " << _upName << " in startDAC()" << std::endl;
  }

  // select the memory as dac data source
  long route = 1;
  ioctl(_upFd, FIOMEMROUTESET, route);

  std::cout << "memrouteset performed on " << _upName << std::endl;
  
  // Clear bit 6 in the DDR Memory Control Register. It is mapped to 
  // mem_dac_run in the MEMORY_APP (dram_dtl.vhd). When 
  // mem_dac_run is set low, the memory counter is reset
  // to MEM2_START_REG.
  ARG_PEEKPOKE pp;
  pp.offset = DDR_MEM_CONTROL;
  ioctl(_ctrlFd, FIOREGGET, &pp);
  // set the DACM fifo reset line (bit 1)
  pp.value = pp.value | 0x0000002;
  ioctl(_ctrlFd, FIOREGSET, &pp);
	
  // Set the dacm fifo reset (bit 1)
  pp.offset = DAC_FIFO_CONTROL;
  ioctl(_ctrlFd, FIOREGGET, &pp);
  pp.value = pp.value & 0x000FFFD;
  ioctl(_ctrlFd, FIOREGSET, &pp);

  // Run the dacm fifo
  pp.offset = DAC_FIFO_CONTROL;
  ioctl(_ctrlFd, FIOREGGET, &pp);
  // Clear the dacm fifo reset (bit 2) so that the fifo can run
  pp.value = pp.value & 0x000FFFD;
  ioctl(_ctrlFd, FIOREGSET, &pp);

  // Set bit 6 in the DDR Memory Control Registered. This will allow the 
  // values in memory bank 2 to be loaded into the DACM fifo, where they will
  // be gated out to the DAC by the tx gate.
  pp.offset = DDR_MEM_CONTROL;
  ioctl(_ctrlFd, FIOREGGET, &pp);
  pp.value = pp.value | 0x0000040;
  ioctl(_ctrlFd, FIOREGSET, &pp);

}

////////////////////////////////////////////////////////////////////////////////////////
void
p7142up::stopDAC() {
  if (_simulate)
      return;

  if (_upFd != -1) {

	  // turn off data routing from mem2
	  long route = 0;
	  ioctl(_upFd, FIOMEMROUTESET, route);

	  // disable NCO in order to stop DAC
	  char config2 = 0;
	  setDACreg(_upFd, 0x03, config2);

	  close(_upFd);
	  _upFd = -1;
  }
}

////////////////////////////////////////////////////////////////////////////////////////
void
p7142up::ncoConfig(double fNCO, double fDAC, char& nco_freq_0, char& nco_freq_1, char& nco_freq_2, char& nco_freq_3) {
    if (_simulate)
        return;
    
	double fNCO_CLK;

	switch (_interpMode) {
	default:
	case 0: // X2  mode
	case 2: // X4L mode
	case 3: // X8  mode
		fNCO_CLK = fDAC / 2;;
		break;
	case 1: // X4 mode
		fNCO_CLK = fDAC;
		break;
	}

	long long freq;

	if ((fNCO/fNCO_CLK) < 0.5)
		freq = (fNCO/fNCO_CLK)*(0x100000000ll);
	else
		/// @todo the following produces a 33 bit number! There is something
		/// wrong with the formula in the DAC datasheet.
		freq = ((fNCO/fNCO_CLK)+1)*(0x100000000ll);


	std::cout << "freq is " << std::hex << freq << std::dec << std::endl;

	nco_freq_0 = (freq >>  0) & 0xff;
	nco_freq_1 = (freq >>  8) & 0xff;
	nco_freq_2 = (freq >> 16) & 0xff;
	nco_freq_3 = (freq >> 24) & 0xff;

}
////////////////////////////////////////////////////////////////////////////////////////
std::string
p7142up::upName() {
	return _upName;
}


