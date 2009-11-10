/*
 * p7142hcr.cpp
 *
 *  Created on: Jan 26, 2009
 *      Author: martinc
 */

#include "p7142hcr.h"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <iostream>
#include <cstdio>
#include "BuiltinGaussian.h"
#include "BuiltinKaiser.h"
#define USE_TIMER
#define IOCTLSLEEPUS 100

using namespace Pentek;

////////////////////////////////////////////////////////////////////////////////////////
p7142hcrdn::p7142hcrdn(std::string devName, int chanId, int gates, int nsum,
		int tsLength, int delay, int prt, int prt2, int pulseWidth,
		bool staggeredPrt, std::string gaussianFile, std::string kaiserFile,
		DDCDECIMATETYPE ddcType, int decimation, bool simulate,
		int simPauseMS, bool internalClock) :
	p7142dn(devName, chanId, decimation, simulate, simPauseMS, gates*tsLength/3, internalClock),
			_gates(gates), _nsum(nsum), _tsLength(tsLength), _delay(delay),
			_prt(prt), _prt2(prt2), _pulseWidth(pulseWidth),
			_staggeredPrt(staggeredPrt), _ddcType(ddcType),
			_gaussianFile(gaussianFile), _kaiserFile(kaiserFile)

{
	std::cout << "downconverter: " << ((_ddcType == Pentek::p7142hcrdn::DDC8DECIMATE) ? "DDC8" : "DDC4") << std::endl;
	std::cout << "decimation:    " << _decimation      << std::endl;
	std::cout << "pulse width:   " << _pulseWidth << std::endl;
	std::cout << "prt:           " << _prt         << std::endl;
	std::cout << "prt2:          " << _prt2        << std::endl;
	std::cout << "staggered:     " << ((_staggeredPrt) ? "true" : "false") << std::endl;
	std::cout << "delay          " << _delay       << std::endl;
	std::cout << "clock source:  " << ((internalClock) ? "internal" : "external") << std::endl;
	std::cout << "ts length:     " << _tsLength    << std::endl;
	std::cout << "gates:         " << _gates       << std::endl;
	std::cout << "nsum:          " << _nsum        << std::endl;


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

	// configure DDC in FPGA
	if (!config()) {
		std::cout << "error initializing filters\n";
	}

}

////////////////////////////////////////////////////////////////////////////////////////
p7142hcrdn::~p7142hcrdn() {
	close(_ctrlFd);
}

////////////////////////////////////////////////////////////////////////////////////////
bool p7142hcrdn::config() {

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
	timerInit();

	if (filterError) {
		return false;
	}

	return true;
}
//////////////////////////////////////////////////////////////////////

int p7142hcrdn::fpgaRepoRevision() {
	_pp.offset = FPGA_REPO_REV;
	ioctl(_ctrlFd, FIOREGGET, &_pp);
	return _pp.value & 0x7fff;

}

//////////////////////////////////////////////////////////////////////
p7142hcrdn::DDCDECIMATETYPE p7142hcrdn::ddc_type() {
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
void p7142hcrdn::startFilters() {

	// Start the DDC  -- do we really want to do this here???
	/// @todo Note that this sets the start bit on channel 0. Doesn't
	/// really belong in this class

	_pp.offset = KAISER_ADDR;
	_pp.value = DDC_START;
	ioctl(_ctrlFd, FIOREGSET, &_pp);
	usleep(IOCTLSLEEPUS);

}

//////////////////////////////////////////////////////////////////////
void p7142hcrdn::stopFilters() {

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
void p7142hcrdn::setGates(int gates) {
	_pp.offset = RADAR_GATES;
	_pp.value = gates;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	ioctl(_ctrlFd, FIOREGGET, &_pp);
}

//////////////////////////////////////////////////////////////////////
void p7142hcrdn::setNsum(int nsum) {
	_pp.offset = CI_NSUM;
	_pp.value = nsum;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	ioctl(_ctrlFd, FIOREGGET, &_pp);
}

//////////////////////////////////////////////////////////////////////
bool p7142hcrdn::loadFilters(FilterSpec& gaussian, FilterSpec& kaiser) {

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

int p7142hcrdn::filterSetup() {

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

bool p7142hcrdn::timerInit() {

#ifdef USE_TIMER
	//
	//    This section initializes the timers.

	double prtClock; // Timer Input Clock Freq
	int periodCount; // Period Count for all Timers
	int PrtScheme; // PRT Scheme for all Timers

	// Internal Timing Setup

	unsigned int ALL_TIMERS = TIMER0 | TIMER1 | TIMER2 | TIMER3 | TIMER4
			| TIMER5 | TIMER6 | TIMER7;

	// Calculate the period and PRT Scheme for dual prt or single prt
	// Note: _prt and _prt2 are expressed in ADC_Clk/2 MHz Counts!
	//       for DDC4: 24 MHz; for DDC8: 62.5 MHz

	int X, Y;
	float prt_ms, prt2_ms;

	switch (_ddcType) {
			case DDC8DECIMATE: {
				prtClock = 62.5e6;
				break;
			}
			case DDC4DECIMATE: {
				prtClock = 24.0e6;
				break;
			}
	}
	if (_staggeredPrt == true) //dual prt
	{
//		prt_ms = (float) _prt / 1e3;
//		prt2_ms = (float) _prt2 / 1e3;

		prt_ms = (float) _prt / prtClock * 1e3;
		prt2_ms = (float) _prt2 / prtClock * 1e3;

		periodCount = (int) (prt_ms * (prt2_ms / prt_ms - (int) (prt2_ms
				/ prt_ms)) / (int) (prt2_ms / prt_ms) * prtClock / 1e3);

		X = (int) ((int) (prt2_ms / prt_ms) / (prt2_ms / prt_ms
				- (int) (prt2_ms / prt_ms)));
		Y = (int) (X * prt2_ms / prt_ms);

		PrtScheme = (Y << 4) | X;
	} else //single prt
	{
//		periodCount = (int) ceil((_prt * prtClock / 10e6));

// 		PRT must be integral multiple of pulsewidth !
		periodCount = _prt;
		PrtScheme = 0x0000;
	}
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

	// TIMER 0
	// Delay Register
	_pp.offset = MT_ADDR; // Address
	_pp.value = DELAY_REG | TIMER0;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	_pp.offset = MT_DATA; // Data
	_pp.value = _delay;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	// Pulse Width Register
	_pp.offset = MT_ADDR; // Address
	_pp.value = WIDTH_REG | TIMER0;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	_pp.offset = MT_DATA; // Data
	_pp.value = _pulseWidth * _gates;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	// For now configure all 8 Timers identically, later we will customize per application
	// TIMER 1
	// Delay Register
	_pp.offset = MT_ADDR; // Address
	_pp.value = DELAY_REG | TIMER1;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	_pp.offset = MT_DATA; // Data
	_pp.value = _delay;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	// Pulse Width Register
	_pp.offset = MT_ADDR; // Address
	_pp.value = WIDTH_REG | TIMER1;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	_pp.offset = MT_DATA; // Data
	_pp.value = _pulseWidth;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	// TIMER 2
	// Delay Register
	_pp.offset = MT_ADDR; // Address
	_pp.value = DELAY_REG | TIMER2;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	_pp.offset = MT_DATA; // Data
	_pp.value = _delay;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	// Pulse Width Register
	_pp.offset = MT_ADDR; // Address
	_pp.value = WIDTH_REG | TIMER2;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	_pp.offset = MT_DATA; // Data
	_pp.value = _pulseWidth;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	// TIMER 3
	// Delay Register
	_pp.offset = MT_ADDR; // A	_delayddress
	_pp.value = DELAY_REG | TIMER3;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	_pp.offset = MT_DATA; // Data
	_pp.value = _delay;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	// Pulse Width Register
	_pp.offset = MT_ADDR; // Address
	_pp.value = WIDTH_REG | TIMER3;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	_pp.offset = MT_DATA; // Data
	_pp.value = _pulseWidth;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	// TIMER 4_delay
	// Delay Register
	_pp.offset = MT_ADDR; // Address
	_pp.value = DELAY_REG | TIMER4;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	_pp.offset = MT_DATA; // Data
	_pp.value = _delay;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	// Pulse Width Register
	_pp.offset = MT_ADDR; // Address
	_pp.value = WIDTH_REG | TIMER4;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	_pp.offset = MT_DATA; // Data
	_pp.value = _pulseWidth;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	// TIMER 5
	// Delay Register
	_pp.offset = MT_ADDR; // Address
	_pp.value = DELAY_REG | TIMER5;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	_pp.offset = MT_DATA; // Data
	_pp.value = _delay;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	// Pulse Width Register
	_pp.offset = MT_ADDR; // Address
	_pp.value = WIDTH_REG | TIMER5;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	_pp.offset = MT_DATA; // Data
	_pp.value = _pulseWidth;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	// TIMER 6
	// Delay Register
	_pp.offset = MT_ADDR; // Address
	_pp.value = DELAY_REG | TIMER6;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	_pp.offset = MT_DATA; // Data
	_pp.value = (int) _delay;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	// Pulse Width Register
	_pp.offset = MT_ADDR; // Address
	_pp.value = WIDTH_REG | TIMER6;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	_pp.offset = MT_DATA; // Data
	_pp.value = _pulseWidth;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	// TIMER 7
	// Delay Register
	_pp.offset = MT_ADDR; // Address
	_pp.value = DELAY_REG | TIMER7;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	_pp.offset = MT_DATA; // Data
	_pp.value = _delay;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	// Pulse Width Register
	_pp.offset = MT_ADDR; // Address_delay
	_pp.value = WIDTH_REG | TIMER7;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	_pp.offset = MT_DATA; // Data
	_pp.value = _pulseWidth;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

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

	// Enable and Trigger All Timers

	// Set Global Enable
	_pp.offset = MT_ADDR; // Address
	_pp.value = PRT_REG | ALL_TIMERS | TIMER_EN;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	usleep(1000);

	// Turn off Write Strobes	_delay
	_pp.offset = MT_WR;
	_pp.value = WRITE_OFF;
	ioctl(_ctrlFd, FIOREGSET, &_pp);
#endif
	return true;

}

/////////////////////////////////////////////////////////////////////////

void p7142hcrdn::startInternalTimer() {
#ifdef USE_TIMER

	//
	//    This start the internal timers.

	unsigned int ALL_TIMERS = TIMER0 | TIMER1 | TIMER2 | TIMER3 | TIMER4
			| TIMER5 | TIMER6 | TIMER7;

	// Turn on Write Strobes
	_pp.offset = MT_WR;
	_pp.value = WRITE_ON;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	// Enable and Trigger All Timers

	_pp.offset = MT_ADDR; // Address
	_pp.value = ALL_TIMERS | ADDR_TRIG;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	usleep(1000);

	// Turn off Write Strobes
	_pp.offset = MT_WR;
	_pp.value = WRITE_OFF;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	// Get current system time as xmit start time
	// setXmitStartTime(microsec_clock::universal_time());
#endif
}

//////////////////////////////////////////////////////////////////////
unsigned short int p7142hcrdn::TTLIn() {

	_pp.offset = TTL_IN;
	ioctl(_ctrlFd, FIOREGGET, &_pp);

	return _pp.value;
}

//////////////////////////////////////////////////////////////////////
void p7142hcrdn::TTLOut(unsigned short int data) {
	_pp.value = data;

	_pp.offset = TTL_OUT1;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

}

//////////////////////////////////////////////////////////////////////
void p7142hcrdn::setInterruptBufSize() {

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
p7142hcrdn::resetDCM(int fd) {
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
void p7142hcrdn::fifoConfig() {
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
#ifdef TIME
void p7142hcrdn::setXmitStartTime(ptime startTime) {
	_xmitStartTime = startTime;
}
#endif

