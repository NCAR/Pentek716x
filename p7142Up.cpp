#include "p7142Up.h"
#include "DDCregisters.h"

#include <cerrno>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>

using namespace Pentek;

////////////////////////////////////////////////////////////////////////////////////////
p7142Up::p7142Up(p7142 * p7142ptr,
        double sampleClockHz, double ncoFreqHz, char cmMode):
        _p7142ptr(*p7142ptr),
        _sampleClockHz(sampleClockHz),
        _ncoFreqHz(ncoFreqHz),
        _cmMode(cmMode),
        _interp(2),
        _mem2depthWords(0),
        _mutex()
{
    boost::recursive_mutex::scoped_lock guard(_mutex);
    
    if (isSimulating())
        return;

    // Note that most of the DAC related initialization is
    // performed in p71xx().

    // Initialize the DAC configuration registers
    initDAC();

    return;

}

////////////////////////////////////////////////////////////////////////////////////////
p7142Up::~p7142Up() {
    boost::recursive_mutex::scoped_lock guard(_mutex);
}

////////////////////////////////////////////////////////////////////////////////////////
bool p7142Up::initDAC() {

    // Set DAC FIFO clock source
    // P7142_DAC_CTRL_STAT_DAC_CLK_SELECT (0x0)
    // P7142_DAC_CTRL_STAT_DAC_CLK_BYPASS (0x8)
    P7142_SET_OUTPUT_CTRL_STAT_DAC_CLK(
    		_p7142ptr._p7142Regs.BAR2RegAddr.dacCtrlStat,
    		P7142_DAC_CTRL_STAT_DAC_CLK_BYPASS);

	// Enable the PLL on the DAC
    // P7142_DAC_CTRL_STAT_PLL_VDD_ENABLE  (0x4)
	// P7142_DAC_CTRL_STAT_PLL_VDD_DISABLE (0x0)
    P7142_SET_OUTPUT_CTRL_STAT_PLL_VDD(
    		_p7142ptr._p7142Regs.BAR2RegAddr.dacCtrlStat,
    		P7142_DAC_CTRL_STAT_PLL_VDD_ENABLE);

    // Choose whether to use or bypass the DCM for the DAC clock
    // P7142_DCM_CTRL_DCM_SEL_IN_CLK (0x0)
    // P7142_DCM_CTRL_DCM_SEL_DCM    (0x8)
    P7142_SET_DCM_CTRL_CLK_SEL(
    		_p7142ptr._p7142Regs.BAR2RegAddr.dcmControl,
    		P7142_DCM_CTRL_DCM_SEL_IN_CLK);


	// Version: set FIR1 to low pass on DAC ChA and ChB, also disable DAC B, if operating at 48 or 125 MHz
    char version =
            1 << 7 |              // DAC A sleep
            0 << 6 |              // DAC B operational
            0 << 5 |              // hplb, DAC B fir1
            0 << 4;               // hpla, DAC A fir1
    setDACreg(DAC5687_VERSION_REG, version);

    // Config 0:
    // Bypass the internal DAC FIFOs since we are using PLL.
    // Set the NCO configuration to high freq,
    // PLL divider = 1,
    // operation mode = X4L.

    char config0;

    switch (int(_sampleClockHz)) {
    case 125000000:
        config0 =
                0 << 6 |               // pll_div
                1 << 5 |               // pll_freq
                0 << 4 |               // pll_kv
                _interp << 2  |        // interpolation mode
                0 << 1 |               // inv_pllock
                1 << 0;                // fifo_bypass
        break;
    case 100000000:
        std::cerr << std::endl;
        std::cerr << "NEED UPCONVERTER SETUP FOR 100 MHz CLOCK!" << std::endl;
        std::cerr << std::endl;
        config0 = _interp << 2;
        break;
    case 48000000:
        config0 =
                1 << 6 |               // pll_div
                0 << 5 |               // pll_freq
                0 << 4 |               // pll_kv
                _interp << 2  |    // interp
                0 << 1 |               // inv_pllock
                1 << 0;                // fifo_bypass
        break;
    default:
        std::cerr << __PRETTY_FUNCTION__ <<
            " has no handling for sample clock @ " << _sampleClockHz <<
            " Hz!" << std::endl;
        exit(1);
    }

    setDACreg(DAC5687_CONFIG0_REG, config0);

    // Config 1: Set input Data two two's complement, non-interleaved
    char config1 = 1 << 4;
    setDACreg(DAC5687_CONFIG1_REG, config1);

    // Config 2: Enable NCO, set cm_mode, enable inv. sync filter
    char config2 = 0x80 | (_cmMode << 1) | 0x1;
    setDACreg(DAC5687_CONFIG2_REG, config2);

    // Config 3: For now just a placeholder
    char config3 = 0x80;
    setDACreg(DAC5687_CONFIG3_REG, config3);

    // Sync Control: Sync NCO, sync coarse mixer, disable FIFO sync
    char sync_cntl = 0x40 | 0x20 | 0x6 << 2;

    setDACreg(DAC5687_SYNC_CNTL_REG, sync_cntl);

    char nco_0;
    char nco_1;
    char nco_2;
    char nco_3;
    ncoConfig(_ncoFreqHz, 4*_sampleClockHz, nco_0, nco_1, nco_2, nco_3);
    //std::cout << std::hex <<
    //        (int)nco_0 << " " <<
    //        (int)nco_1 << " " <<
    //        (int)nco_2 << " " <<
    //        (int)nco_3 << " " <<
    //        std::dec << std::endl;

    setDACreg(DAC5687_NCO_FREQ_0_REG,       nco_0);
    setDACreg(DAC5687_NCO_FREQ_1_REG,       nco_1);
    setDACreg(DAC5687_NCO_FREQ_2_REG,       nco_2);
    setDACreg(DAC5687_NCO_FREQ_3_REG,       nco_3);

    setDACreg(DAC5687_SER_DATA_1_REG,       0xff);

    setDACreg(DAC5687_DACA_GAIN_0_REG,      0xff);
    setDACreg(DAC5687_DACB_GAIN_0_REG,      0xff);
    setDACreg(DAC5687_DACA_DACB_GAIN_1_REG, 0xff);

    //std::cout << "DAC registers after configuration " << std::endl;
    //dumpDACregs();

    std::cout << "sample clock:     " << _sampleClockHz << std::endl;
    std::cout << "nco frequency:    " << _ncoFreqHz << std::endl;
    std::cout << "coarse mixer mode:" << (int)_cmMode << std::endl;

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////
bool
p7142Up::isSimulating() const {
    boost::recursive_mutex::scoped_lock guard(_mutex);
    return _p7142ptr.isSimulating();
}

////////////////////////////////////////////////////////////////////////////////////////
void
p7142Up::dumpDACregs() {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (isSimulating()) {
        std::cout << "No DAC registers: running in simulation mode." << std::endl;
        return;
    }

    for (int i = 0; i < 32; i++) {
        // get value
        char val = getDACreg(i<<8);
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
p7142Up::getDACreg(int reg) {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (isSimulating())
        return 0;

    char val = (char)P7142Dac5686ReadReg (&_p7142ptr._p7142Regs.BAR2RegAddr, reg);

    return val;
}

////////////////////////////////////////////////////////////////////////////////////////
void
p7142Up::setDACreg(int reg, char val) {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (isSimulating())
        return;

    P7142Dac5686WriteReg (&_p7142ptr._p7142Regs.BAR2RegAddr, reg, val);
    usleep(1);
}

////////////////////////////////////////////////////////////////////////////////////////
void
p7142Up::write(int32_t* data, int n) {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (isSimulating())
        return;

    // memory depth in 4 byte words
    _mem2depthWords = n;

    // write to memory bank 2
    int writeStatus = _p7142ptr.memWrite(2, data, 4*n);

    if (writeStatus < 0) {
    	std::cerr << "DMA write to memory bank 2 failed, with status code: " << writeStatus << std::endl;
    }

    return;

    // the following code can be enabled if you want to verify that the memory writes are working
    std::vector<int32_t> readBack;
    readBack.resize(n);

    int readStatus = _p7142ptr.memRead(2, &readBack[0], 4*n);

    if (readStatus < 0) {
    	std::cerr << "DMA read from memory bank 2 failed, with status code: " << readStatus << std::endl;
    }

    for (int i = 0; i < n; i++) {
    	std::cout << i << ": " << std::hex << data[i] << " " << readBack[i] << std::dec << std::endl;
    }

}


////////////////////////////////////////////////////////////////////////////////////////
void p7142Up::startDAC() {
	boost::recursive_mutex::scoped_lock guard(_mutex);

	if (isSimulating())
		return;

	uint32_t memreg;
	uint32_t temp;

	// Some notes:
	// In the memory to DAC mode, data is clocked out of memory bank2
	// into the DACM fifo, and then clocked out of there to the DAC.
	// It turms out that DACM shares configuration lines with
	// the regular DAC fifo. So, to control the DACM fifo, you
	// make calls to ReadyFlow which appear to be accessing the regular
	// DAC fifo (via dacFifo.FifoCtrl).
	//
	// The memory and dacm control logic are found in dram_dtl.vhd.
	//
	// Don't forget to usleep() after changing the control bits.
	// It took about three weeks of fooling around to discover why
	// the memory counter was not being initialized to zero. The previous
	// code did not have this issue, since it accessed locations via
	// ioctls, which probably had enough overhead to allow preceeding
	// signal changed to be latched by gateflow.

	// set the memory depth
	P7142_SET_DDR_MEM_DEPTH(
			_p7142ptr._p7142Regs.BAR2RegAddr.ddrMem.ddrMemBankDepth[2].Lsb,
			_mem2depthWords);
	usleep(1);

	// set the memory address start value
	P7142_SET_DDR_MEM_RW_ADDR(
			_p7142ptr._p7142Regs.BAR2RegAddr.ddrMem.ddrMemBankRwAddr[2].Lsb, 0);
	usleep(1);

	// select memory bank 2 => dacm fifo => DAC mode
	P7142_SET_DDR_MEM_MODE( _p7142ptr._p7142Regs.BAR2RegAddr.ddrMem.ddrMemCtrl,
			P7142_DDR_MEM_BANK_2_DAC_OUTPUT_MODE);
	usleep(1);

	// Disable memory bank 2 (bit D06). This is mapped to mem_dac_run in
	// dram_dtl.vhd. Turns out that setting it low resets the mem2 address
	// counter.
	P7142_REG_READ(_p7142ptr._p7142Regs.BAR2RegAddr.ddrMem.ddrMemCtrl, memreg);
	temp = memreg & ~0x0040;
	P7142_REG_WRITE(_p7142ptr._p7142Regs.BAR2RegAddr.ddrMem.ddrMemCtrl, temp);
	usleep(1);
	// Memory address counter has been reset. Re-enable memory 2.
	temp = memreg | 0x0040;
	P7142_REG_WRITE(_p7142ptr._p7142Regs.BAR2RegAddr.ddrMem.ddrMemCtrl, temp);
	usleep(1);

	// disable the dacm fifo.
	P7142_SET_FIFO_CTRL_FIFO_ENABLE(
			_p7142ptr._p7142Regs.BAR2RegAddr.dacFifo.FifoCtrl,
			P7142_FIFO_DISABLE);
	usleep(1);

	// put dacm fifo into reset
	P7142_SET_FIFO_CTRL_RESET(
			_p7142ptr._p7142Regs.BAR2RegAddr.dacFifo.FifoCtrl,
			P7142_FIFO_RESET_HOLD);
	usleep(1);

	// bring dacm fifo out of reset
	P7142_SET_FIFO_CTRL_RESET(
			_p7142ptr._p7142Regs.BAR2RegAddr.dacFifo.FifoCtrl,
			P7142_FIFO_RESET_RELEASE);
	usleep(1);

	// re-enable dacm fifo
	P7142_SET_FIFO_CTRL_FIFO_ENABLE(
			_p7142ptr._p7142Regs.BAR2RegAddr.dacFifo.FifoCtrl,
			P7142_FIFO_ENABLE);
	usleep(1);

}

////////////////////////////////////////////////////////////////////////////////////////
void
p7142Up::stopDAC() {
  boost::recursive_mutex::scoped_lock guard(_mutex);

  if (isSimulating())
      return;

  // Turn off data routing from mem2
  P7142_SET_DDR_MEM_MODE(
  		_p7142ptr._p7142Regs.BAR2RegAddr.ddrMem.ddrMemCtrl,
  		P7142_DDR_MEM_DISABLE_MODE);

  // disable the NCO in order to stop the DAC
  char config2 = (_cmMode << 1) | 0x1;
  setDACreg(DAC5687_CONFIG2_REG, config2);

}

////////////////////////////////////////////////////////////////////////////////////////
void
p7142Up::ncoConfig(double fNCO, double fDAC, char& nco_freq_0, char& nco_freq_1, char& nco_freq_2, char& nco_freq_3) {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (isSimulating())
        return;
    
    double fNCO_CLK;

    switch (_interp) {
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

    nco_freq_0 = (freq >>  0) & 0xff;
    nco_freq_1 = (freq >>  8) & 0xff;
    nco_freq_2 = (freq >> 16) & 0xff;
    nco_freq_3 = (freq >> 24) & 0xff;

    // std::cout << "freq is " << std::hex << freq << std::dec << std::endl;

}

////////////////////////////////////////////////////////////////////////////////////////
double
p7142Up::sampleClockHz() {
	return _sampleClockHz;
}

