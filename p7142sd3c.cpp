/*
 * p7142sd3c.cpp
 *
 *  Created on: Jan 26, 2009
 *      Author: martinc
 */

#include "p7142sd3c.h"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <iostream>
#include <cmath>
#include <cstdio>
#include "BuiltinGaussian.h"
#include "BuiltinKaiser.h"
#define IOCTLSLEEPUS 100

using namespace Pentek;
using namespace boost::posix_time;

////////////////////////////////////////////////////////////////////////////////////////
p7142sd3cdn::p7142sd3cdn(
    std::string devName, 
    int chanId, 
    int gates, 
    int nsum,
    int tsLength, 
    double rx_delay, 
    double tx_delay, 
    double prt, 
    double prt2, 
    double pulseWidth,
    bool staggeredPrt, 
    std::vector<double> timer_delays,
    std::vector<double> timer_widths,
    bool freeRun, 
    std::string gaussianFile, 
    std::string kaiserFile,
    bool simulate, 
    double simPauseMS,
    int simWaveLength,
    bool internalClock) :
        p7142dn(devName, 
                chanId, 
                1, 
                simulate, 
                simWaveLength,
                internalClock),
        _gates(gates), 
        _nsum(nsum), 
        _tsLength(tsLength),
        _staggeredPrt(staggeredPrt), 
        _freeRun(freeRun),
        _gaussianFile(gaussianFile), 
        _kaiserFile(kaiserFile),
        _simPauseMS(simPauseMS),
        _simPulseNum(0),
        _simWaitCounter(0),
        _lastPulse(0),
        _droppedPulses(0),
        _syncErrors(0),
        _firstBeam(true)
{
    assert(timer_delays.size() == 5 && timer_widths.size() == 5);
    
    // determine the operating mode
    if (_freeRun) {
    	_mode = FR;
    } else {
    	if (_nsum > 1) {
    		_mode = CI;
    	} else {
    		_mode = PT;
    	}
    }

    // set up page and mask registers for FIOREGSET and FIOREGGET functions to access FPGA registers
    _pp.page = 2; // PCIBAR 2
    _pp.mask = 0;
    
    // Open the control device before we call any methods which access the card
    _openControlDevice();
    
    // Query the firmware to get DDC type
    _ddcType = ddc_type();

    // Set the ADC clock rate based on DDC type
    _adc_clock = (_ddcType == DDC4DECIMATE) ? 48.0e6 : 125.0e6;
    // Convert prt, prt2, pulseWidth, and delay into our local representation, 
    // which is in units of (ADC clock counts / 2)
    _prt = lround(prt * _adc_clock / 2);
    _prt2 = lround(prt2 * _adc_clock / 2);
    _prf = 1.0 / prt;
    _prf2 = 1.0 / prt2;
    
    int rxDelayCounts    = lround(rx_delay   * _adc_clock / 2);
    int pulseWidthCounts = lround(pulseWidth * _adc_clock / 2);
    int txDelayCounts    = lround(tx_delay   * _adc_clock / 2);

    // sync pulse timer
    _timer_delays.push_back(0);
    _timer_widths.push_back(4);
    
    // rx gate timer
    _timer_delays.push_back(rxDelayCounts);
    _timer_widths.push_back(pulseWidthCounts*_gates);
    
    // tx pulse pulse timer
    _timer_delays.push_back(txDelayCounts);
    _timer_widths.push_back(pulseWidthCounts);
    
    // the 5 general purpose timers
    for (int i = 0; i < 5; i++) {
        _timer_delays.push_back(lround(timer_delays[i] * _adc_clock / 2));
        _timer_widths.push_back(lround(timer_widths[i] * _adc_clock / 2));
    }

    if (_simulate) {
        // we generate simulated data with no coherent integration
        // (i.e., nsum == 1) and with pulse tagging (i.e., freeRun == false).
        if (_nsum > 1) {
            std::cerr <<
                "p7142sd3cdn: nsum is forced to one when simulating data\n" <<
                std::endl;
            _nsum = 1;
        }
    }
    
    // initialize the buffering scheme.
    initBuffer();

	std::cout << "downconverter: " << ((_ddcType == Pentek::p7142sd3cdn::DDC8DECIMATE) ? "DDC8" : "DDC4") << std::endl;
    std::cout << "rx delay:      " << _timer_delays[1] << " adc_clock/2 counts"  << std::endl; 
    std::cout << "rx gate width: " << _timer_widths[1] << " adc_clock/2 counts"   << std::endl;
    std::cout << "tx delay:      " << _timer_delays[2] << " adc_clock/2 counts"  << std::endl;
    std::cout << "tx pulse width:" << _timer_widths[2] << " adc_clock/2 counts"   << std::endl;
	std::cout << "gate spacing:  " << gateSpacing()    << " m"                    << std::endl;
    std::cout << "prt:           " << _prt             << " adc_clock/2 counts"   << std::endl;
	std::cout << "prt2:          " << _prt2            << " adc_clock/2 counts"   << std::endl;
	std::cout << "staggered:     " << ((_staggeredPrt) ? "true" : "false")        << std::endl;
	std::cout << "rng to gate0:  " << rangeToFirstGate() << " m"                  << std::endl;
	std::cout << "clock source:  " << (usingInternalClock() ? "internal" : "external") << std::endl;
	std::cout << "ts length:     " << _tsLength                                   << std::endl;
	std::cout << "gates:         " << _gates                                      << std::endl;
	std::cout << "nsum:          " << _nsum                                       << std::endl;
	std::cout << "free run:      " << ((_freeRun) ? "true" : "false")             << std::endl;
	std::cout << "adc clock:     " << _adc_clock       << " Hz"                   << std::endl;
	std::cout << "prf:           " << _prf             << " Hz"                   << std::endl;
	std::cout << "data rate:     " << dataRate()/1.0e3 << " KB/s"                 << std::endl;
	std::cout << "sim usleep     " << _simPauseMS*1000 << "us"                    <<std::endl;
	for (int i = 0; i < 8; i++) {
		std::cout << "timer " << i << " delay: " << _timer_delays[i] << " adc_clock/2 counts"  << std::endl;
		std::cout << "timer " << i << " width: " << _timer_widths[i] << " adc_clock/2 counts"  << std::endl;
	}

	if (_simulate)
		return;

	if (fpgaRepoRevision() == 0) {
		std::cerr << "**** Warning: The FPGA firmware revision number is zero. Was the correct firmware loaded?"
		<< std::endl;
	}

	// verify that the correct down converter was selected.
	if (ddc_type() != _ddcType) {
		std::cerr << "The firmware DDC type (ddc4 or ddc8) does not match the specified type. Program will terminate."
		<< std::endl;
		exit(1);
	}

    /// The SD3C Pentek firmware requires that bypass divider value be set to
    /// 2 * (pulse width in adc_frequency counts).
    setBypassDivider(2 * _timer_widths[2]);
	std::cout << "bypass decim:  " << bypassDivider()  << std::endl;
    
	// Note the fpga firmware revision
	std::cout << "FPGA revision: " << fpgaRepoRevision()
			<< std::endl;

	// stop the timers
	timersStartStop(false);

	// flush the fifos. Note that a flush must not be issued
	// after the timers have been configured, as this will zero
	// the timer parameters.
	flush();

	// configure DDC in FPGA
	if (!config()) {
		std::cout << "error initializing filters\n";
	}

}

////////////////////////////////////////////////////////////////////////////////////////
p7142sd3cdn::~p7142sd3cdn() {
	close(_ctrlFd);
	delete [] _buf;
}

////////////////////////////////////////////////////////////////////////////////////////
void
p7142sd3cdn::_openControlDevice() {
    if (_simulate)
        return;
    
	_ctrlFd = open(_devCtrl.c_str(), O_RDWR);
	if (_ctrlFd < 0) {
		std::cout << "unable to open Pentek ctrl device" << std::endl;
		exit(1);
	}
}

////////////////////////////////////////////////////////////////////////////////////////
bool p7142sd3cdn::config() {

	// reset the FPGA clock managers. Necessary since some of our
	// new DCMs in the firmware use the CLKFX output, which won't
	// lock at startup.
	resetDCM(_ctrlFd);

	// configure the fifo
	fifoConfig();

	// Stop the filters from running
	stopFilters();

	// set number of gates
	setGates(_gates);

	// set number of coherent integrator sums
	setNsum(_nsum);

	// Is coherent integration enabled?
	std::cout << "coherent integration is " <<
	      (_nsum > 1 ? "enabled" : "disabled") << std::endl;

	// set up the filters. Will do nothing if either of
	// the filter file paths is empty.
	bool filterError = filterSetup();

	// initialize the internal timers
	initTimers();

	// set free run mode as appropriate
	freeRunConfig();

	if (filterError) {
		return false;
	}

	return true;
}

//////////////////////////////////////////////////////////////////////
void p7142sd3cdn::freeRunConfig() {
    if (_simulate)
        return;

	// set the free run bit as needed

	// get the current state of transceiver control register
	_pp.offset = TRANS_CNTRL;
	ioctl(_ctrlFd, FIOREGGET, &_pp);
	usleep(IOCTLSLEEPUS);

	if (_freeRun) {
		// set free run
		_pp.value = _pp.value | TRANS_FREE_RUN;
	} else {
		// clear free run
		_pp.value = _pp.value & ~TRANS_FREE_RUN;
	}

	// write transceiver control register
	ioctl(_ctrlFd, FIOREGSET, &_pp);
	usleep(IOCTLSLEEPUS);

	ioctl(_ctrlFd, FIOREGGET, &_pp);
	usleep(IOCTLSLEEPUS);

	std::cout << "transceiver control register is " << _pp.value << std::endl;
}

//////////////////////////////////////////////////////////////////////
int p7142sd3cdn::fpgaRepoRevision() {
    if (_simulate)
        return 0;
    
	_pp.offset = FPGA_REPO_REV;
	ioctl(_ctrlFd, FIOREGGET, &_pp);
	return _pp.value & 0x7fff;

}

//////////////////////////////////////////////////////////////////////
p7142sd3cdn::DDCDECIMATETYPE p7142sd3cdn::ddc_type() {
    if (_simulate)
        return DDC8DECIMATE;
    
	_pp.offset = FPGA_REPO_REV;
    ioctl(_ctrlFd, FIOREGGET, &_pp);
    
	DDCDECIMATETYPE ddctype = DDC4DECIMATE;
	switch (_pp.value & 0x8000) {
	case 0x8000:
		ddctype = DDC8DECIMATE;
		break;
	case 0x0000:
		ddctype = DDC4DECIMATE;
		break;
	}

	return ddctype;

}

//////////////////////////////////////////////////////////////////////
void p7142sd3cdn::startFilters() {

	// Start the DDC  -- do we really want to do this here???
	/// @todo Note that this sets the start bit on channel 0. Doesn't
	/// really belong in this class

	_pp.offset = KAISER_ADDR;
	_pp.value = DDC_START;
	ioctl(_ctrlFd, FIOREGSET, &_pp);
	usleep(IOCTLSLEEPUS);

	std::cout << "filters enabled on " << _dnName << std::endl;
}

//////////////////////////////////////////////////////////////////////
void p7142sd3cdn::stopFilters() {
    if (_simulate)
        return;

	// stop the filters if they are running.
	_pp.offset = KAISER_ADDR;
	ioctl(_ctrlFd, FIOREGGET, &_pp);
	_pp.offset = KAISER_ADDR;
	_pp.value = DDC_STOP;
	ioctl(_ctrlFd, FIOREGSET, &_pp);
	usleep(IOCTLSLEEPUS);
	_pp.offset = KAISER_ADDR;
	ioctl(_ctrlFd, FIOREGGET, &_pp);

}

//////////////////////////////////////////////////////////////////////
void p7142sd3cdn::setGates(int gates) {
    if (_simulate)
        return;
    
	_pp.offset = RADAR_GATES;
	_pp.value = gates;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	ioctl(_ctrlFd, FIOREGGET, &_pp);
}

//////////////////////////////////////////////////////////////////////
void p7142sd3cdn::setNsum(int nsum) {
    if (_simulate)
        return;
    
	_pp.offset = CI_NSUM;
	_pp.value = nsum;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	ioctl(_ctrlFd, FIOREGGET, &_pp);
}

//////////////////////////////////////////////////////////////////////
bool p7142sd3cdn::loadFilters(FilterSpec& gaussian, FilterSpec& kaiser) {
    if (_simulate)
        return true;

	bool kaiserLoaded;
	bool gaussianLoaded;

	int attempt;

	// program kaiser coefficients

	int ddcSelect = _chanId << 14;
	attempt = 0;

	do {
		kaiserLoaded = true;
		for (unsigned int i = 0; i < kaiser.size(); i++) {
			unsigned int readBack;

			int ramAddr = 0;
			int ramSelect = 0;
			switch (_ddcType) {
			case DDC8DECIMATE:
				ramAddr = i / 8;
				ramSelect = (i % 8) << 4;
				break;
			case DDC4DECIMATE:
				ramAddr = i / 4;
				ramSelect = (i % 4) << 4;
				break;
			}
			_pp.value = ddcSelect | DDC_STOP | ramSelect | ramAddr;
			_pp.offset = KAISER_ADDR;
			ioctl(_ctrlFd, FIOREGSET, &_pp);
			usleep(IOCTLSLEEPUS);

			// write the value
			// LS word first
			_pp.value = kaiser[i] & 0xFFFF;
			_pp.offset = KAISER_DATA_LSW;
			ioctl(_ctrlFd, FIOREGSET, &_pp);
			usleep(IOCTLSLEEPUS);

			// then the MS word -- since coefficients are 18 bits and FPGA registers are 16 bits!
			_pp.value = (kaiser[i] >> 16) & 0x3;
			_pp.offset = KAISER_DATA_MSW;
			ioctl(_ctrlFd, FIOREGSET, &_pp);
			usleep(IOCTLSLEEPUS);

			// latch coefficient
			_pp.value = 0x1;
			_pp.offset = KAISER_WR;
			ioctl(_ctrlFd, FIOREGSET, &_pp);
			usleep(IOCTLSLEEPUS);

			// disable writing (kaiser readback only succeeds if we do this)
			_pp.value = 0x0;
			_pp.offset = KAISER_WR;
			ioctl(_ctrlFd, FIOREGSET, &_pp);
			usleep(IOCTLSLEEPUS);

			// read back the programmed value; we need to do this in two words as above.
			_pp.offset = KAISER_READ_LSW;
			ioctl(_ctrlFd, FIOREGGET, &_pp);
			usleep(IOCTLSLEEPUS);

			readBack = _pp.value;
			_pp.offset = KAISER_READ_MSW;
			ioctl(_ctrlFd, FIOREGGET, &_pp);
			usleep(IOCTLSLEEPUS);

			readBack |= (_pp.value << 16);
			if (readBack != kaiser[i]) {
				std::cout << "kaiser readback failed for coefficient "
						<< std::dec << i << std::hex << ", wrote " << kaiser[i]
						<< ", read " << readBack << std::endl;

				kaiserLoaded = false;
			} else {
				// std::cout << "programmed kaiser " << i << std::endl;
			}

		}
		attempt++;
	} while (!kaiserLoaded && attempt < 1); // was 50

	if (kaiserLoaded) {
		std::cout << kaiser.size()
				<< " Kaiser filter coefficients succesfully loaded" << std::endl;
	} else {
		std::cout << "Unable to load the Kaiser filter coefficients" << std::endl;
	}

	// program gaussian coefficients
	attempt = 0;

	// Note that the DDC select is accomplished in the kaiser filter coefficient
	// address register, which was done during the previous kaiser filter load.
	do {
		gaussianLoaded = true;
		for (unsigned int i = 0; i < gaussian.size(); i++) {

			unsigned int readBack;
			int ramAddr = 0;
			int ramSelect = 0;
			switch (_ddcType) {
			case DDC8DECIMATE:
				ramAddr = i % 8;
				ramSelect = (i / 8) << 4;
				break;
			case DDC4DECIMATE:
				ramAddr = i % 12;
				ramSelect = (i / 12) << 4;
				break;
			}
			/// @todo early versions of the gaussian filter programming required
			/// the ds select bits to be set in the gaussian address register.
			/// We can take this out when we get a working bitstream with this
			/// fixed
			_pp.value = ddcSelect | ramSelect | ramAddr;
			_pp.offset = GUASSIAN_ADDR;

			// set the address
			ioctl(_ctrlFd, FIOREGSET, &_pp);
			usleep(IOCTLSLEEPUS);

			// write the value
			// LS word first
			_pp.value = gaussian[i] & 0xFFFF;
			_pp.offset = GUASSIAN_DATA_LSW;
			ioctl(_ctrlFd, FIOREGSET, &_pp);
			usleep(IOCTLSLEEPUS);
			// then the MS word -- since coefficients are 18 bits and FPGA registers are 16 bits!
			_pp.value = (gaussian[i] >> 16) & 0x3;
			_pp.offset = GUASSIAN_DATA_MSW;
			ioctl(_ctrlFd, FIOREGSET, &_pp);
			usleep(IOCTLSLEEPUS);

			// enable writing
			_pp.value = 0x1;
			_pp.offset = GUASSIAN_WR;
			ioctl(_ctrlFd, FIOREGSET, &_pp);
			usleep(IOCTLSLEEPUS);

			// disable writing (gaussian readback only succeeds if we do this)
			_pp.value = 0x0;
			_pp.offset = GUASSIAN_WR;
			ioctl(_ctrlFd, FIOREGSET, &_pp);
			usleep(IOCTLSLEEPUS);

			// read back the programmed value; we need to do this in two words as above.
			_pp.offset = GUASSIAN_READ_LSW;
			ioctl(_ctrlFd, FIOREGGET, &_pp);
			usleep(IOCTLSLEEPUS);
			readBack = _pp.value;
			_pp.offset = GUASSIAN_READ_MSW;
			ioctl(_ctrlFd, FIOREGGET, &_pp);
			usleep(IOCTLSLEEPUS);
			readBack |= _pp.value << 16;
			if (readBack != gaussian[i]) {
				std::cout << "gaussian readback failed for coefficient "
						<< std::dec << i << std::hex << ", wrote "
						<< gaussian[i] << ", read " << readBack << std::endl;

				gaussianLoaded = false;
			} else {
				// std::cout << "programmed gaussian " << i << std::endl;
			}
		}
		attempt++;
	} while (!gaussianLoaded && attempt < 1); //was 50

	if (gaussianLoaded) {
		std::cout << gaussian.size()
				<< " Gaussian filter coefficients succesfully loaded" << std::endl;
	} else {
		std::cout << "Unable to load the Gaussian filter coefficients" << std::endl;
	}

	// return to decimal output
	std::cout << std::dec;

	return kaiserLoaded && gaussianLoaded;

}
////////////////////////////////////////////////////////////////////////

int p7142sd3cdn::filterSetup() {

	// get the gaussian filter coefficients.
	FilterSpec gaussian;
	if (_gaussianFile.size() != 0) {
		FilterSpec g(_gaussianFile);
		if (!g.ok()) {
			std::cerr << "Incorrect or unaccessible filter definition: "
					<< _gaussianFile << std::endl;
			return -1;
		} else {
			gaussian = g;
		}
	} else {
		std::string gaussianFilterName;
		BuiltinGaussian builtins;
		// The pulsewidth expressed in microseconds must match one of those
		// available in BuiltinGaussian.
		double pulseWidthUs = 1.00;
		gaussianFilterName = "ddc8_1_0";

		// Choose the correct builtin Gaussian filter coefficient set.
		switch (_ddcType) {
		case DDC8DECIMATE: {
			switch ((int)(_timer_widths[2]/62.5 * 10)) { // pulse width in 62.5 MHz counts

			case 2:                             //pulse width = 0.256 microseconds
				pulseWidthUs = 0.256;
				gaussianFilterName = "ddc8_0_2";
				break;
			case 3:								//pulse width = 0.384 microseconds
				pulseWidthUs = 0.384;
				gaussianFilterName = "ddc8_0_3";
				break;
			case 5:
				pulseWidthUs = 0.512;			//pulse width = 0.512 microseconds
				gaussianFilterName = "ddc8_0_5";
				break;
			case 6:								//pulse width = 0.64 microseconds
				pulseWidthUs = 0.64;
				gaussianFilterName = "ddc8_0_6";
				break;
			case 7:								//pulse width = 0.768 microseconds
				pulseWidthUs = 0.768;
				gaussianFilterName = "ddc8_0_7";
				break;
			case 8:								//pulse width = 0.896 microseconds
				pulseWidthUs = 0.896;
				gaussianFilterName = "ddc8_0_8";
				break;
			case 10:							//pulse width = 1.024 microseconds
				pulseWidthUs = 1.024;
				gaussianFilterName = "ddc8_1_0";
				break;
			default:
				std::cerr << "chip width specification of " << _timer_widths[2]
						  << " is not recognized, filter will be configured for a "
						  << pulseWidthUs << " uS pulse\n";
				break;
			}
			break;
		}
		case DDC4DECIMATE: {    // pulse_widht in 24 MHz counts
			pulseWidthUs = 1.0;
			gaussianFilterName = "ddc4_1_0";
			break;
		}
		}

		if (builtins.find(gaussianFilterName) == builtins.end()) {
			std::cerr << "No entry for " << gaussianFilterName << ", "
					<< pulseWidthUs
					<< " us pulsewidth in the list of builtin Gaussian filters!"
					<< std::endl;
			abort();
		}
		gaussian = FilterSpec(builtins[gaussianFilterName]);
		std::cout << "Using gaussian filter coefficient set "
				<< gaussianFilterName << std::endl;
	}

	// get the kaiser filter coefficients
	std::string kaiserFilterName;
	FilterSpec kaiser;
	double kaiserBandwidth = 5.0;
	if (_kaiserFile.size() != 0) {
		FilterSpec k(_kaiserFile);
		if (!k.ok()) {
			std::cerr << "Incorrect or unaccessible filter definition: "
					<< _kaiserFile << std::endl;
			return -1;
		} else {
			kaiser = k;
		}
	} else {
		BuiltinKaiser builtins;
		std::string kaiserFilterName;
		switch (_ddcType) {
		case DDC8DECIMATE: {
			kaiserFilterName = "ddc8_5_0";
			break;
		}
		case DDC4DECIMATE: {
			kaiserFilterName = "ddc4_4_0";
			break;
		}
		}
		if (builtins.find(kaiserFilterName) == builtins.end()) {
			std::cerr << "No entry for " << kaiserFilterName
					<< " in the list of builtin Kaiser filters!" << std::endl;
			abort();
		}
		kaiser = FilterSpec(builtins[kaiserFilterName]);
		std::cout << "Using kaiser filter coefficient set " << kaiserFilterName
				<< std::endl;
	}

	std::cout << "Kaiser filter will be programmed for " << kaiserBandwidth
			<< " MHz bandwidth\n";

	// load the filter coefficients

	if (!loadFilters(gaussian, kaiser)) {
		std::cerr << "Unable to load filters\n";
		return -1;
	}

	return 0;
}

/////////////////////////////////////////////////////////////////////////

bool p7142sd3cdn::initTimers() {
    if (_simulate)
        return true;

	//
	//    This section initializes the timers.

	int periodCount; // Period Count for all Timers
	int PrtScheme; // PRT Scheme for all Timers

	// Internal Timing Setup

	unsigned int ALL_TIMERS = TIMER0 | TIMER1 | TIMER2 | TIMER3 | TIMER4
			| TIMER5 | TIMER6 | TIMER7;

	// Calculate the period and PRT Scheme for dual prt or single prt
	// Note: _prt and _prt2 are expressed in ADC_Clk/2 MHz Counts!
	//       for DDC4: 24 MHz; for DDC8: 62.5 MHz

    double prtClock = _adc_clock / 2; // Timer Input Clock Freq
	int X, Y;
	float prt_ms, prt2_ms;

	if (_staggeredPrt == true) {
		// dual prt
		prt_ms = (float) _prt / prtClock * 1e3;
		prt2_ms = (float) _prt2 / prtClock * 1e3;

		periodCount = (int) (prt_ms * (prt2_ms / prt_ms - (int) (prt2_ms
				/ prt_ms)) / (int) (prt2_ms / prt_ms) * prtClock / 1e3);

		X = (int) ((int) (prt2_ms / prt_ms) / (prt2_ms / prt_ms
				- (int) (prt2_ms / prt_ms)));
		Y = (int) (X * prt2_ms / prt_ms);

		PrtScheme = (Y << 4) | X;
	} else {
		//single prt
		// 	PRT must be integral multiple of pulsewidth !
		periodCount = _prt;
		PrtScheme = 0x0000;
	}

	std::cout << "periodCount is " << periodCount << std::endl;

	// Control Register
	_pp.offset = MT_ADDR;
	_pp.value = CONTROL_REG | ALL_TIMERS;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	// Enable Timer
	_pp.offset = MT_DATA;
	_pp.value = TIMER_ON;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	// Turn on Write Strobes
	_pp.offset = MT_WR;
	_pp.value = WRITE_ON;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	// Configure the timers
	std::vector<TimerSetup> timers;
    
    // the sync pulse
    timers.push_back(TimerSetup(TIMER0, _timer_delays[0], _timer_widths[0]));
    // the rx gate
    timers.push_back(TimerSetup(TIMER1, _timer_delays[1], _timer_widths[1]));
    // the tx pulse
    timers.push_back(TimerSetup(TIMER2, _timer_delays[2], _timer_widths[2]));
    // 1st general purpose timer
    timers.push_back(TimerSetup(TIMER3, _timer_delays[3], _timer_widths[3]));
    // 2nd general purpose timer
    timers.push_back(TimerSetup(TIMER4, _timer_delays[4], _timer_widths[4]));
    // 3rd general purpose timer
    timers.push_back(TimerSetup(TIMER5, _timer_delays[5], _timer_widths[5]));
    // 4th general purpose timer
    timers.push_back(TimerSetup(TIMER6, _timer_delays[6], _timer_widths[6]));
    // 5th general purpose timer
    timers.push_back(TimerSetup(TIMER7, _timer_delays[7], _timer_widths[7]));

	for (unsigned int i = 0; i < timers.size(); i++) {
		
		// Delay Register
		_pp.offset = MT_ADDR; // Address
		_pp.value = DELAY_REG | timers[i].id;
		ioctl(_ctrlFd, FIOREGSET, &_pp);

		_pp.offset = MT_DATA; // Data
		_pp.value = timers[i].delay;
		ioctl(_ctrlFd, FIOREGSET, &_pp);

		// Pulse Width Register
		_pp.offset = MT_ADDR; // Address
		_pp.value = WIDTH_REG | timers[i].id;
		ioctl(_ctrlFd, FIOREGSET, &_pp);

		_pp.offset = MT_DATA; // Data
		_pp.value = timers[i].width;
		ioctl(_ctrlFd, FIOREGSET, &_pp);
	}

	// ALL TIMERS
	// Period Register
	_pp.offset = MT_ADDR; // Address
	_pp.value = PERIOD_REG | ALL_TIMERS;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	_pp.offset = MT_DATA; // Data
	_pp.value = periodCount;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	//Multiple PRT Register
	_pp.offset = MT_ADDR; // Address
	_pp.value = PRT_REG | ALL_TIMERS;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	_pp.offset = MT_DATA; // Mult PRT Valu Timer 0
	_pp.value = PrtScheme;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	// Turn off Write Strobes	_delay
	_pp.offset = MT_WR;
	_pp.value = WRITE_OFF;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	return true;

}

/////////////////////////////////////////////////////////////////////////
void p7142sd3cdn::timersStartStop(bool start) {
    if (_simulate)
        return;
	//
	//    This start the internal timers.
	bool INTERNAL_TRIG = true;
	unsigned int ALL_TIMERS = TIMER0 | TIMER1 | TIMER2 | TIMER3 | TIMER4
			| TIMER5 | TIMER6 | TIMER7;

	// Turn on Write Strobes
	_pp.offset = MT_WR;
	_pp.value = WRITE_ON;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	// Control Register
	_pp.offset = MT_ADDR;
	_pp.value = CONTROL_REG | ALL_TIMERS;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	// Enable/Disable Timer
	_pp.offset = MT_DATA;
	if (start) {
		_pp.value = TIMER_ON;
	} else {
		_pp.value = 0;
	}
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	_pp.offset = MT_ADDR; // Address
	if (start) {
		if (INTERNAL_TRIG)
			_pp.value = ALL_TIMERS | ADDR_TRIG;  // internal trigger
		else
			_pp.value = ALL_TIMERS | GPS_EN;     // external trigger
	} else {
		std::cout << "timer stopped\n";
		_pp.value = ALL_TIMERS;
	}
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	if (start) {
		_pp.value = ALL_TIMERS;
		ioctl(_ctrlFd, FIOREGSET, &_pp);
	}

	// Turn off Write Strobes
	_pp.offset = MT_WR;
	_pp.value = WRITE_OFF;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	if (start) {
		std::cout << "timers started" << std::endl;
	} else {
		std::cout << "timers stopped" << std::endl;
		//exit (1);
	}

	// Get current system time as xmit start time
	setXmitStartTime(microsec_clock::universal_time());
}

//////////////////////////////////////////////////////////////////////
unsigned short int p7142sd3cdn::TTLIn() {
    if (_simulate)
        return 0;

	_pp.offset = TTL_IN;
	ioctl(_ctrlFd, FIOREGGET, &_pp);

	return _pp.value;
}

//////////////////////////////////////////////////////////////////////
void p7142sd3cdn::TTLOut(unsigned short int data) {
    if (_simulate)
        return;

    _pp.value = data;

	_pp.offset = TTL_OUT1;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

}

//////////////////////////////////////////////////////////////////////
void p7142sd3cdn::setInterruptBufSize() {

	// how many bytes are there in each time series?
	int tsBlockSize;
	if (_nsum < 2) {
		tsBlockSize = _tsLength * _gates * 2 * 2;
	} else {
		// coherently integrated data has:
		// 4 tags followed by even IQ pairs followed by odd IQ pairs,
		// for all gates. Tags, I and Q are 4 byte integers.
		tsBlockSize = _tsLength * (4 + _gates * 2 * 2) * 4;
	}

	double pulseFreq = 1.0 / (_prt / (10.0e6));
	double tsFreq = pulseFreq / _tsLength;

	// we want the interrupt buffer size to be a multiple of tsBlockSize,
	// but no more than 20 interrupts per second.
	int intBlocks = 1;

	if (tsFreq <= 20) {
		intBlocks = 1;
	} else {
		intBlocks = (tsFreq / 20) + 1;
	}

	int bufferSize = tsBlockSize * intBlocks;

	std::cout << "prt is " << _prt << "  prt frequency is " << pulseFreq
			<< "  ts freq is " << tsFreq << "  tsblocks per interrupt is "
			<< intBlocks << std::endl;

	std::cout << "pentek interrupt buffer size is " << bufferSize << std::endl;

	// set the buffer size
	bufset(_dnFd, bufferSize, 2);

}
////////////////////////////////////////////////////////////////////////////////////////
void
p7142sd3cdn::resetDCM(int fd) {
    if (_simulate)
        return;

    _pp.offset = DCM_CONTROL;

	// read the dcm control register
	ioctl(_ctrlFd, FIOREGGET, &_pp);
	//std::cout << "DCM control readback is 0x" << std::hex << _pp.value << std::endl;

	// turn on the DCM reset bit
	_pp.value = 0x10 | _pp.value;
	ioctl(_ctrlFd, FIOREGSET, &_pp);
	usleep(1000);

	ioctl(_ctrlFd, FIOREGGET, &_pp);
	//std::cout << "DCM control readback is 0x" << std::hex << _pp.value << std::endl;

	// turn off the DCM reset bit
	_pp.value = _pp.value & ~0x10;
	ioctl(_ctrlFd, FIOREGSET, &_pp);
	usleep(1000);

	ioctl(_ctrlFd, FIOREGGET, &_pp);
	//std::cout << "DCM control readback is 0x" << std::hex << _pp.value << std::endl;

}

//////////////////////////////////////////////////////////////////////
void p7142sd3cdn::fifoConfig() {
    if (_simulate)
        return;

    // The fifos need to be configured for
	// the given channel that we are using.

	// find the fifo configuration register
	unsigned int readBack;
	int ppOffset = ADC_FIFO_CTRL_1;
	switch (_chanId) {
	case 0:
		ppOffset = ADC_FIFO_CTRL_1;
		break;
	case 1:
		ppOffset = ADC_FIFO_CTRL_2;
		break;
	case 2:
		ppOffset = ADC_FIFO_CTRL_3;
		break;
	case 3:
		ppOffset = ADC_FIFO_CTRL_4;
		break;
	}

	_pp.offset = ppOffset;
	ioctl(_ctrlFd, FIOREGGET, &_pp);
	readBack = _pp.value;

	// And configure ADC FIFO Control for this channel
	_pp.offset = ppOffset;
	_pp.value = readBack & 0x000034BF;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

}

//////////////////////////////////////////////////////////////////////
void p7142sd3cdn::setXmitStartTime(ptime startTime) {
	_xmitStartTime = startTime;
}

//////////////////////////////////////////////////////////////////////
ptime p7142sd3cdn::timeOfPulse(unsigned long pulseNum) const {
    // Figure out offset since transmitter start based on the pulse
    // number and PRT(s).
    double offsetSeconds;
    if (_staggeredPrt) {
        unsigned long prt1Count = pulseNum / 2 + pulseNum % 2;
        unsigned long prt2Count = pulseNum / 2;
        offsetSeconds =  prt1Count /_prf + prt2Count / _prf2;
    } else {
        offsetSeconds = pulseNum / _prf;
    }
    // Translate offsetSeconds to a boost::posix_time::time_duration
    double remainder = offsetSeconds;
    int hours = remainder / 3600;
    remainder -= (3600 * hours);
    int minutes = remainder / 60;
    remainder -= (60 * minutes);
    int seconds = remainder;
    remainder -= seconds;
    int nanoseconds = 1.0e9 * remainder;
    int fractionalSeconds = nanoseconds *
        (time_duration::ticks_per_second() / 1.0e9);
    time_duration offset(hours, minutes, seconds,
            fractionalSeconds);
    // Finally, add the offset to the _xmitStartTime to get the absolute
    // pulse time
    return(_xmitStartTime + offset);
}

//////////////////////////////////////////////////////////////////////
int p7142sd3cdn::dataRate() {
	int rate;
	if (_nsum < 2) {
		// no coherent integration
		// there is a four byte pulse tag at the beginning of each pulse; thus
		// it looks like an extra gate in terms of data rate. There is a two byte
		// I and a two byte Q (total 4 bytes) for each range gate.
		rate = _prf * (_gates+1) * 4;
	} else {
		// coherent integration
		// there is a 16 byte tag at the beginning of each pulse. Each pulse
		// returns a set of even I's and Q's, and a set of odd I's and Q's. The
		// even and odd pulses are separated by the prt, and so taken together they
		// run at half the prf. Each I and Q for a gate is 32 bits (wider than the
		// non-CI mode becasue they are sums of 16 bit numbers), so there are 8 bytes
		// per gate for even and 8 bytes per gate for odd pulses.
		rate = (_prf/2)*(16+_gates*8*2)/_nsum;
	}
	return (int) rate;

}

//////////////////////////////////////////////////////////////////////////////////
//
// ******    Buffer management and data handling in the following section    *****
//
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
int
p7142sd3cdn::read(char* buf, int n) {
    // Unless we're simulating, we just use the superclass read
    if (!_simulate) {
        return p7142dn::read(buf, n);
    }

    // ************ simulation mode *************

    // Generate simulated data
    makeSimData(n);

    // copy to user buffer
    for (int i = 0; i < n; i++) {
    	buf[i] = _simFifo[0];
    	_simFifo.pop_front();
    }

    return n;
}

//////////////////////////////////////////////////////////////////////////////////

char*
p7142sd3cdn::getBeam(unsigned int& pulsenum) {

	// perform the simulation wait if necessary
	if (_simulate) {
		simWait();
	}

	if (_nsum <= 1) {
		if (_freeRun) {
			// free run mode
			pulsenum = 0;
			return frBeam();
		} else {
			// pulse tagger mode
			return ptBeamDecoded(pulsenum);
		}
	} else {
		// coherent integration mode
		return ciBeam(pulsenum);
	}
}

//////////////////////////////////////////////////////////////////////////////////
int
p7142sd3cdn::beamLength() {
	return _beamLength;
}

//////////////////////////////////////////////////////////////////////////////////
char*
p7142sd3cdn::ptBeamDecoded(unsigned int& pulseNum) {

	// get the beam
	char* buf = ptBeam();

	// unpack the channel number and pulse sequence number.
	// Unpack the 4-byte channel id/pulse number
	unsigned int chan;
	_unpackChannelAndPulse(buf, chan, pulseNum);
	if (int(chan) != _chanId) {
		std::cerr << "p7142sd3cdnThread for channel " << _chanId <<
				" got data for channel " << chan << "!" << std::endl;
		abort();
	}

	// Initialize _lastPulse if this is the first pulse we've seen
	if (_firstBeam) {
		_lastPulse = pulseNum - 1;
		_firstBeam = false;
	}

	// How many pulses since the last one we saw?
	int delta = pulseNum - _lastPulse;
	if (delta < (-MAX_PULSE_NUM / 2))
		delta += (MAX_PULSE_NUM + 1); // pulse number wrapped

	if (delta == 0) {
		std::cerr << "Channel " << _chanId << ": got repeat of pulse " <<
				pulseNum << "!" << std::endl;
		abort();
	} else if (delta != 1) {
		std::cerr << _lastPulse << "->" << pulseNum << ": ";
		if (delta < 0) {
			std::cerr << "Channel " << _chanId << " went BACKWARD " <<
				-delta << " pulses" << std::endl;
		} else {
			std::cerr << "Channel " << _chanId << " dropped " <<
				delta - 1 << " pulses" << std::endl;
		}
	}
	_droppedPulses += (delta - 1);
	_lastPulse = pulseNum;

	return buf+4;
}
//////////////////////////////////////////////////////////////////////////////////
char*
p7142sd3cdn::ptBeam() {

	while(1) {
		if (_firstBeam) {
			// skip over the first 4 bytes, assuming that
			// they are a good sync word. _firstBeam will
			// be set false by
			read(_buf, 4);
		}
		// read pulse tag and one beam into buf
		read(_buf, _beamLength+4);

		// read the next sync word
		char syncbuf[4];
		read(syncbuf, 4);
		uint32_t sync;
		sync = *((uint32_t*)syncbuf);

		if (sync == SD3C_SYNCWORD) {
			return _buf;
		} else {
			_syncErrors++;
			// scan byte by byte for sync word
			int s = 0;
			while(1) {
				memmove(syncbuf, syncbuf+1,3);
				read(syncbuf+3, 1);
				sync = *((uint32_t*)syncbuf);
				if (sync == SD3C_SYNCWORD) {
					break;
				}
			}
		}
	}
}

//////////////////////////////////////////////////////////////////////////////////
char*
p7142sd3cdn::ciBeam(unsigned int& pulseNum) {
	while(1) {
		if (_firstBeam) {
			// skip over the first 16 bytes, assuming that
			// they are a good tag word. _firstBeam will
			// be set false by
			read(_buf, 16);
		}
		// read one beam into buf
		read(_buf, _beamLength);

		// read the next tag word
		char tagbuf[16];
		read(tagbuf, 16);

		if (ciCheckTag(tagbuf, pulseNum)) {
			return _buf;
		} else {
			_syncErrors++;
			// scan byte by byte for the
			int s = 0;
			while(1) {
				memmove(tagbuf, tagbuf+1,15);
				read(tagbuf+15, 1);
				// check for synchronization
				if (ciCheckTag(tagbuf, pulseNum)) {
					break;
				}
			}
		}
	}
}

//////////////////////////////////////////////////////////////////////////////////
char*
p7142sd3cdn::frBeam() {
	read(_buf, _beamLength);
	return _buf;
}

//////////////////////////////////////////////////////////////////////////////////
bool
p7142sd3cdn::ciCheckTag(char* p, unsigned int& pulseNum) {
	pulseNum = 123456;
	return true;
}

//////////////////////////////////////////////////////////////////////////////////
void
p7142sd3cdn::initBuffer() {

	// note that _beamLength is only the length of the
	// IQ data (in bytes)

	if (_nsum <= 1) {
	      if (_freeRun) {
	    	  // free run mode has:
	    	  //   16 bit I and Q pairs
	    	  _beamLength = _gates * 2 * 2;
	      } else {
	      // pulse tag mode has:
	      //    4-byte sync word and a 4 byte pulse tag
	      //    followed by 16 bit I and Q pairs
	    	  _beamLength = _gates * 2 * 2;
	      }
	  } else {
	      // coherent integration mode has:
	      //   4 4-byte tags followed by
		  //   even 32 bit I and Q pairs followed by
		  //   odd  32 bit I and Q pairs,
	      // for all gates. Is and Qs are 4 byte integers.
    	  _beamLength = _gates * 2 * 2 * 4;
	  }

	// allocate the buffer to hold one beam of IQ data
	_buf = new char[_beamLength];
}

//////////////////////////////////////////////////////////////////////////////////
void
p7142sd3cdn::makeSimData(int n) {

	while(_simFifo.size() < (unsigned int)n) {
		switch(_mode) {
		case FR:{
			// ************* free run mode ***************
			for (int i = 0; i < _beamLength/4; i++) {
				uint32_t iq;
				char* p = (char*)&iq;
				p7142dn::read(p, 4);
				for (int j = 0; j < 4; j++) {
					_simFifo.push_back(p[j]);
				}
			}
			break;
		}
		case PT: {
			// ********** pulse tag mode **************
			// Add sync word
			static uint32_t syncword = SD3C_SYNCWORD;
			for (int i = 0; i < 4; i++) {
				_simFifo.push_back(((char*)&syncword)[i]);
			}
			// Add the pulse tag for this sample:
			//       bits 31:30  Channel number         0-3 (2 bits)
			//       bits 29:00  Pulse sequence number  0-1073741823 (30 bits)
			// This is packed as a little-endian order 4-byte word
			unsigned int channel = _chanId;
			uint32_t tag = (channel << 30) | (_simPulseNum++ & 0x3fffffff);
			char* p = (char*)&tag;
			for (int i = 0; i < 4; i++) {
				_simFifo.push_back(p[i]);
			}
			// Add IQ data. Occasionally drop some data
			bool doBadSync = ((1.0 * rand())/RAND_MAX) < 5.0e-6;
			int nPairs = _beamLength/4;
			if (doBadSync) {
				nPairs = (int)(((1.0 * rand())/RAND_MAX) * nPairs);
			}
			for (int i = 0; i < nPairs; i++) {
				uint32_t iq;
				char* p = (char*)&iq;
				p7142dn::read(p, 4);
				for (int j = 0; j < 4; j++) {
					_simFifo.push_back(p[j]);
				}
			}
			break;
		}
		case CI: {
			/// Add the coherent integration tag for this sample:
			/// @todo Actually synthesize the correct tags
			uint32_t tag = 1;
			char* p = (char*)&tag;
			for (int j = 0; j < 4; j++) {
				for (int i = 0; i < 4; i++) {
					_simFifo.push_back(p[i]);
				}
			}

			// Add IQ data.
			int nPairs = _beamLength/8;
			for (int i = 0; i < nPairs; i++) {
				char iq[8];
				// first i
				p7142dn::read(iq, 8);
				for (int j = 0; j < 8; j++) {
					_simFifo.push_back(iq[j]);
				}
			}
			break;
		}
		}
	}
}

//////////////////////////////////////////////////////////////////////////////////
void
p7142sd3cdn::simWait() {
	// because the usleep overhead is large, sleep every 100 calls
	if (!(_simWaitCounter++ % 100)) {
		usleep((int)(100*_simPauseMS*1000));
	}
}
//////////////////////////////////////////////////////////////////////////////////
void
p7142sd3cdn::_unpackChannelAndPulse(const char* buf, unsigned int & chan,
        unsigned int & pulseNum) {
    // Channel number is the upper two bits of the channel/pulse num word, which
    // is stored in little-endian byte order
    const unsigned char *ubuf = (const unsigned char*)buf;
    chan = (ubuf[3] >> 6) & 0x3;

    // Pulse number is the lower 30 bits of the channel/pulse num word, which is
    // stored in little-endian byte order
    pulseNum = (ubuf[3] & 0x3f) << 24 | ubuf[2] << 16 | ubuf[1] << 8 | ubuf[0];
}

//////////////////////////////////////////////////////////////////////////////////
unsigned long
p7142sd3cdn::droppedPulses() {
	unsigned long retval = _droppedPulses;
	return retval;
}

//////////////////////////////////////////////////////////////////////////////////
unsigned long
p7142sd3cdn::syncErrors() {
	unsigned long retval = _syncErrors;
	return retval;
}

//////////////////////////////////////////////////////////////////////////////////
void
p7142sd3cdn::dumpSimFifo(std::string label, int n) {
	std::cout << label <<  " _simFifo length: " << _simFifo.size() << std::endl;
	std::cout << std::hex;
	for (int i = 0; i < n && i < _simFifo.size(); i++) {
		std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)(unsigned char)_simFifo[i] << " ";
		if (!((i+1) % 40)) {
			std::cout << std::endl;
		}
	}
	std::cout << std::dec << std::endl;;
}



