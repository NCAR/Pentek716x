/*
 * p7142Up.cpp
 *
 *  Created on: Oct 12, 2010
 *      Author: burghart
 */
#include "p7142Up.h"
#include "p7142.h"
#include "DDCregisters.h"

#include <cerrno>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>

using namespace Pentek;

////////////////////////////////////////////////////////////////////////////////////////
p7142Up::p7142Up(p7142 * myP7142Ptr, std::string upName,
        double sampleClockHz, double ncoFreqHz, char mode):
        _p7142(*myP7142Ptr),
        _sampleClockHz(sampleClockHz),
        _ncoFreqHz(ncoFreqHz),
        _interpMode(2),
        _upName(upName),
        _mem2Name(""),
        _upFd(-1),
        _mem2depth(0),
        _mutex()
{
    boost::recursive_mutex::scoped_lock guard(_mutex);
    
    if (isSimulating())
        return;

    // create the up device names
    _upName = _p7142.devName() + "/up/" + _upName;
    _mem2Name = _p7142.devName() + "/mem2";

    // open upconverter
    _upFd = open(_upName.c_str(), O_RDONLY);
    if (_upFd < 0) {
        std::cerr << "unable to open " << _upName << ": " << strerror(errno) << 
            std::endl;
        exit(1);
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
            1 << 7 |              // DAC A sleep
            0 << 6 |              // DAC B operational
            0 << 5 |              // hplb, DAC B fir1
            0 << 4;              // hpla, DAC A fir1
    setDACreg(_upFd, 0x0, version);

    // Config 0:
    // Bypass DAC FIFOs since we are using PLL,
    // set NCO to high freq,
    // PLL divider = 1,
    // interp = X4L
    char config0;
    
    switch (int(_sampleClockHz)) {
    case 125000000:
        config0 =
                0 << 6 |               // pll_div
                1 << 5 |               // pll_freq
                0 << 4 |               // pll_kv
                _interpMode << 2  |    // interp
                0 << 1 |               // inv_pllock
                1 << 0;                // fifo_bypass
        break;
    case 100000000:
        std::cerr << std::endl;
        std::cerr << "NEED UPCONVERTER SETUP FOR 100 MHz CLOCK!" << std::endl;
        std::cerr << std::endl;
        config0 = _interpMode << 2;
        break;
    case 48000000:
        config0 =
                1 << 6 |               // pll_div
                0 << 5 |               // pll_freq
                0 << 4 |               // pll_kv
                _interpMode << 2  |    // interp
                0 << 1 |               // inv_pllock
                1 << 0;                // fifo_bypass
        break;
    default:
        std::cerr << __PRETTY_FUNCTION__ << 
            " has no handling for sample clock @ " << _sampleClockHz << 
            " Hz!" << std::endl;
        exit(1);    
    }

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
}

////////////////////////////////////////////////////////////////////////////////////////
p7142Up::~p7142Up() {
    boost::recursive_mutex::scoped_lock guard(_mutex);
    if (_upFd >= 0) {
        std::cout << __FUNCTION__ << " closing upconverter" << std::endl;
        close (_upFd);
    }
}

////////////////////////////////////////////////////////////////////////////////////////
bool
p7142Up::isSimulating() const {
    boost::recursive_mutex::scoped_lock guard(_mutex);
    return _p7142.isSimulating();
}

////////////////////////////////////////////////////////////////////////////////////////
void
p7142Up::dumpDACregs(int fd) {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (isSimulating()) {
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
p7142Up::getDACreg(int fd, int reg) {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (isSimulating())
        return 0;

    ARG_PEEKPOKE pp;
    pp.offset = reg;
    pp.page = 0;
    pp.mask = 0;

    int status = ioctl(fd, FIOREGGET, (long)&pp);
    if (status < 0) {
        perror("FIOREGGET ioctl error");
    }

    return(pp.value);
}

////////////////////////////////////////////////////////////////////////////////////////
void
p7142Up::setDACreg(int fd, int reg, char val) {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (isSimulating())
        return;

    ARG_PEEKPOKE pp;
    pp.offset = reg;
    pp.page = 0;
    pp.mask = 0;
    pp.value = val;

    int status = ioctl(fd, FIOREGSET, (long)&pp);
    if (status < 0) {
        perror("FIOREGSET ioctl error");
    }
}

////////////////////////////////////////////////////////////////////////////////////////
void
p7142Up::write(long* data, int n) {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (isSimulating())
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
p7142Up::startDAC() {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (isSimulating())
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
    ioctl(_p7142.ctrlFd(), FIOREGGET, &pp);
    // set the DACM fifo reset line (bit 1)
    pp.value = pp.value | 0x0000002;
    ioctl(_p7142.ctrlFd(), FIOREGSET, &pp);

    // Set the dacm fifo reset (bit 1)
    pp.offset = DAC_FIFO_CONTROL;
    ioctl(_p7142.ctrlFd(), FIOREGGET, &pp);
    pp.value = pp.value & 0x000FFFD;
    ioctl(_p7142.ctrlFd(), FIOREGSET, &pp);

    // Run the dacm fifo
    pp.offset = DAC_FIFO_CONTROL;
    ioctl(_p7142.ctrlFd(), FIOREGGET, &pp);
    // Clear the dacm fifo reset (bit 2) so that the fifo can run
    pp.value = pp.value & 0x000FFFD;
    ioctl(_p7142.ctrlFd(), FIOREGSET, &pp);

    // Set bit 6 in the DDR Memory Control Registered. This will allow the 
    // values in memory bank 2 to be loaded into the DACM fifo, where they will
    // be gated out to the DAC by the tx gate.
    pp.offset = DDR_MEM_CONTROL;
    ioctl(_p7142.ctrlFd(), FIOREGGET, &pp);
    pp.value = pp.value | 0x0000040;
    ioctl(_p7142.ctrlFd(), FIOREGSET, &pp);

}

////////////////////////////////////////////////////////////////////////////////////////
void
p7142Up::stopDAC() {
  boost::recursive_mutex::scoped_lock guard(_mutex);

  if (isSimulating())
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
p7142Up::ncoConfig(double fNCO, double fDAC, char& nco_freq_0, char& nco_freq_1, char& nco_freq_2, char& nco_freq_3) {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (isSimulating())
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
        freq = (long long)((fNCO/fNCO_CLK)*(0x100000000ll));
    else
        /// @todo the following produces a 33 bit number! There is something
        /// wrong with the formula in the DAC datasheet.
        freq = (long long)(((fNCO/fNCO_CLK)+1)*(0x100000000ll));


    std::cout << "freq is " << std::hex << freq << std::dec << std::endl;

    nco_freq_0 = (freq >>  0) & 0xff;
    nco_freq_1 = (freq >>  8) & 0xff;
    nco_freq_2 = (freq >> 16) & 0xff;
    nco_freq_3 = (freq >> 24) & 0xff;

}
