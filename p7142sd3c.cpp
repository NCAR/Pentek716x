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
#include <cstdio>
#include "BuiltinGaussian.h"
#include "BuiltinKaiser.h"
#define IOCTLSLEEPUS 100

using namespace Pentek;
using namespace boost::posix_time;

////////////////////////////////////////////////////////////////////////////////////////
p7142sd3cdn::p7142sd3cdn(std::string devName, int chanId, int gates, int nsum,
		int tsLength, int delay, int prt, int prt2, int pulseWidth,
		bool staggeredPrt, bool freeRun, std::string gaussianFile, std::string kaiserFile,
		DDCDECIMATETYPE ddcType, int decimation, bool simulate,
		int simPauseMS, bool internalClock) :
	p7142dn(devName, chanId, decimation, simulate, simPauseMS, gates*tsLength/3, internalClock),
			_gates(gates), _nsum(nsum), _tsLength(tsLength),
			_prt(prt), _prt2(prt2), _pulseWidth(pulseWidth),
			_delay(delay), _staggeredPrt(staggeredPrt), _freeRun(freeRun),
			_ddcType(ddcType), _gaussianFile(gaussianFile),
			_kaiserFile(kaiserFile), _simPulseNum(0)

{

	_adc_clock = (_ddcType == DDC4DECIMATE) ? 48.0e6 : 125.0e6;
	_prf = (_adc_clock / 2)/ _prt;
    _prf2 = (_adc_clock / 2) / _prt2;


	if (_simulate) {
        // we generate simulated data with no coherent integration
        // (i.e., nsum == 1) and with pulse tagging (i.e., freeRun == false).
        if (_freeRun) {
            std::cerr <<
                "p7142sd3cdn: freeRun forced to false when simulating data\n" <<
                std::endl;
            _freeRun = false;
        }
        if (_nsum > 1) {
            std::cerr <<
                "p7142sd3cdn: nsum is forced to one when simulating data\n" <<
                std::endl;
            _nsum = 1;
        }
    }
	std::cout << "downconverter: " << ((_ddcType == Pentek::p7142sd3cdn::DDC8DECIMATE) ? "DDC8" : "DDC4") << std::endl;
	std::cout << "decimation:    " << _decimation      << " adc_clock counts"     << std::endl;
	std::cout << "pulse width:   " << _pulseWidth      << " adc_clock/2 counts"   << std::endl;
	std::cout << "gate spacing:  " << gateSpacing()    << " m"                    << std::endl;
	std::cout << "prt:           " << _prt             << " adc_clock/2 counts"   << std::endl;
	std::cout << "prt2:          " << _prt2            << " adc_clock/2 counts"   << std::endl;
	std::cout << "staggered:     " << ((_staggeredPrt) ? "true" : "false")        << std::endl;
	std::cout << "delay:         " << _delay           << " adc clock/2 counts"   << std::endl;
	std::cout << "rng to gate0:  " << rangeToFirstGate() << " m"                  << std::endl;
	std::cout << "clock source:  " << ((internalClock) ? "internal" : "external") << std::endl;
	std::cout << "ts length:     " << _tsLength                                   << std::endl;
	std::cout << "gates:         " << _gates                                      << std::endl;
	std::cout << "nsum:          " << _nsum                                       << std::endl;
	std::cout << "free run:      " << ((_freeRun) ? "true" : "false")             << std::endl;
	std::cout << "adc clock:     " << _adc_clock       << " Hz"                   << std::endl;
	std::cout << "prf:           " << _prf             << " Hz"                   << std::endl;
	std::cout << "data rate:     " << dataRate()/1.0e3 << " KB/s"                 << std::endl;


	if (_simulate)
		return;

	// set up page and mask registers for FIOREGSET and FIOREGGET functions to access FPGA registers
	_pp.page = 2; // PCIBAR 2
	_pp.mask = 0;

	// open Pentek 7142 ctrl device
	_ctrlFd = open(_devCtrl.c_str(), O_RDWR);
	if (_ctrlFd < 0) {
		std::cout << "unable to open Ctrl device\n";
		return;
	}

	// check the fpga firmware revision
	std::cout << "FPGA revision: " << fpgaRepoRevision()
			<< std::endl;

	if (fpgaRepoRevision() == 0) {
		std::cerr << "**** Warning: The FPGA firmware revision number is zero. Was the correct firmware loaded?"
		<< std::endl;
	}

	// verify that the correct down converter was selected.
	if (ddc_type() != ddcType) {
		std::cerr << "The firmware DDC type (ddc4 or ddc8) does not match the specified type. Program will terminate."
		<< std::endl;
		exit(1);
	}

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
	_pp.offset = FPGA_REPO_REV;
	ioctl(_ctrlFd, FIOREGGET, &_pp);
	return _pp.value & 0x7fff;

}

//////////////////////////////////////////////////////////////////////
p7142sd3cdn::DDCDECIMATETYPE p7142sd3cdn::ddc_type() {
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
	_pp.offset = RADAR_GATES;
	_pp.value = gates;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	ioctl(_ctrlFd, FIOREGGET, &_pp);
}

//////////////////////////////////////////////////////////////////////
void p7142sd3cdn::setNsum(int nsum) {
	_pp.offset = CI_NSUM;
	_pp.value = nsum;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	ioctl(_ctrlFd, FIOREGGET, &_pp);
}

//////////////////////////////////////////////////////////////////////
bool p7142sd3cdn::loadFilters(FilterSpec& gaussian, FilterSpec& kaiser) {

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
				<< " Kaiser filter coefficients succesfully loaded\n";
	} else {
		std::cout << "Unable to load the Kaiser filter coefficients\n";
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
				<< " Gaussian filter coefficients succesfully loaded\n";
	} else {
		std::cout << "Unable to load the Gaussian filter coefficients\n";
	}

	// return to decimal output
	std::cout << std::dec << std::endl;

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
			switch ((int)(_pulseWidth/62.5 * 10)) { // pulse width in 62.5 MHz counts

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
				std::cerr << "chip width specification of " << _pulseWidth
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

	//
	//    This section initializes the timers.

	int periodCount; // Period Count for all Timers
	int PrtScheme; // PRT Scheme for all Timers

	std::cout << "delay is " << _delay << std::endl;
	std::cout << "pulseWidth is " << _pulseWidth << std::endl;

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
	timers.push_back(TimerSetup(TIMER0, 0,      4));
	// the rx gate
	timers.push_back(TimerSetup(TIMER1, _delay, _pulseWidth*_gates));
	// the tx gate
	timers.push_back(TimerSetup(TIMER2, _delay, _pulseWidth));
	// general purpose timers
	timers.push_back(TimerSetup(TIMER3, _delay+5, _pulseWidth+16));
	timers.push_back(TimerSetup(TIMER4, _delay, _pulseWidth));
	timers.push_back(TimerSetup(TIMER5, _delay, _pulseWidth));
	timers.push_back(TimerSetup(TIMER6, _delay, _pulseWidth));
	timers.push_back(TimerSetup(TIMER7, _delay, _pulseWidth));

	for (unsigned int i = 0; i < timers.size(); i++) {
		// TIMER 1
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

	_pp.offset = TTL_IN;
	ioctl(_ctrlFd, FIOREGGET, &_pp);

	return _pp.value;
}

//////////////////////////////////////////////////////////////////////
void p7142sd3cdn::TTLOut(unsigned short int data) {
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

//////////////////////////////////////////////////////////////////////
int
p7142sd3cdn::read(char* buf, int bufsize) {
    // Unless we're simulating, we just use the superclass read
    if (!_simulate)
        return p7142dn::read(buf, bufsize);

    // The rest is for generating simulated data.  We use p7142dn::read()
    // to get the simulated Is and Qs, but we add tags for each time series
    // sample, as we would see from SD3C.

    // Code below assumes 32-bit ints and 16-bit shorts!
    assert(sizeof(int) == 4);
    assert(sizeof(short) == 2);

    // We expect a specific bufsize to hold _tsLength samples
    // of _gates gates, 2-byte I and Q, and a 4-byte tag for
    // each sample
    assert(bufsize = (_tsLength * (4 * _gates + 4)));

    // Static buffer for simulated IQ data from p7142dn::read()
    static char *iqData = 0;
    int iqDataLen = _tsLength * 4 * _gates;
    if (! iqData)
        iqData = new char[iqDataLen];
    // Get the simulated Is and Qs from ::read()
    p7142dn::read(iqData, iqDataLen);

    // Build our tagged samples
    char* bufp = buf;
    for (int ts = 0; ts < _tsLength; ts++) {
        // Create the tag for this sample.  Currently, this uses the
        // profiler no-coherent-integration 32-bit tag (defined as of
        // 2009-12-17):
        //       bits 31:30  Channel number         0-3 (2 bits)
        //       bits 29:00  Pulse sequence number  0-1073741823 (30 bits)
        // This is packed as a little-endian order 4-byte word!
        unsigned int channel = _chanId;
        unsigned int tag = (channel << 30) | (_simPulseNum++ & 0x3fffffff);
        memcpy(bufp, &tag, 4);  // copy the 4-byte tag to the head of bufp
        bufp += 4;  // move past the 4-byte tag we just added
        _bytesRead += 4;
        // copy _gates Is and Qs for this sample
        memcpy(bufp, iqData + ts * 4 * _gates, 4 * _gates);
        bufp += 4 * _gates;
    }
    assert(bufp == (buf + bufsize));

    return bufsize;
}
