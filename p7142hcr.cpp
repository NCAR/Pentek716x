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
#include <iomanip.h>
#include <stdio.h>
#include "BuiltinGaussian.h"
#include "BuiltinKaiser.h"

using namespace Pentek;

////////////////////////////////////////////////////////////////////////////////////////
p7142hcrdn::p7142hcrdn(std::string devName, int chanId, int gates,
		   int delay, int prt, int prt2, int pulse_width, bool stgr_prt,
		   std::string gaussianFile, std::string kaiserFile,int bypdivrate,
		   bool simulate, int simPauseMS):
p7142dn(devName, chanId, bypdivrate, simulate, simPauseMS),
_gates(gates),
_delay(delay),
_prt(prt),
_pulse_width(pulse_width),
_stgr_prt(stgr_prt),
_gaussianFile(gaussianFile),
_kaiserFile(kaiserFile)


{
	if (_simulate)
		return;

	// set up page and mask registers for FIOREGSET and FIOREGGET functions to access FPGA registers
	_pp.page = 2;  // PCIBAR 2
	_pp.mask = 0;

	// open Pentek 7142 ctrl device
	_ctrlFd = open(_devCtrl.c_str(), O_RDWR);
	if (_ctrlFd < 0) {
		std::cout << "unable to open Ctrl device\n";
		return;
	}

	// configure DDC in FPGA
	if (!config())
		std::cout << "error initializing filters\n";

	_pp.offset = KAISER_ADDR;
	ioctl(_ctrlFd, FIOREGGET, &_pp);

}

////////////////////////////////////////////////////////////////////////////////////////
p7142hcrdn::~p7142hcrdn() {
	close(_ctrlFd);
}

////////////////////////////////////////////////////////////////////////////////////////
bool
p7142hcrdn::config() {

	_pp.offset = KAISER_ADDR;
	ioctl(_ctrlFd, FIOREGGET, &_pp);

	// stop the filters if they are running.
	_pp.offset = KAISER_ADDR;
	_pp.value = DDC_STOP;
    ioctl(_ctrlFd, FIOREGSET, &_pp);
	usleep(100000);

	_pp.offset = KAISER_ADDR;
	ioctl(_ctrlFd, FIOREGGET, &_pp);

	// Reset Decimator clocks
//	Adapter_Write32(&_chanAdapter, V4, DEC_RST_REG, RST_ACT);
//	usleep(100000);
//	Adapter_Write32(&_chanAdapter, V4, DEC_RST_REG, RST_CLR);
//	usleep(100000);

    // Configure ADC FIFO Control Registers

	int ppOffset = ADC_FIFO_CTRL_1;

	switch(_chanId) {
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

	unsigned int readBack;

	_pp.offset = ppOffset;
	ioctl(_ctrlFd, FIOREGGET, &_pp);
	readBack = _pp.value;

	_pp.offset = ppOffset;
	_pp.value = readBack & 0x000034BF;
    ioctl(_ctrlFd, FIOREGSET, &_pp);

	// set up the filters. Will do nothing if either of
	// the filter file paths is empty.
	if (filterSetup()) {
		// error initializing the filters
		return false;
	}

	usleep(100000);


	// initialize the timers
//	if (!timerInit())
//		return -1;

	return true;
}
//////////////////////////////////////////////////////////////////////

void p7142hcrdn::startFilters() {

	// Start the DDC  -- do we really want to do this here???
	/// @todo Note that this sets the start bit on channel 0. Doesn't
	/// really belong in this class

	_pp.offset = KAISER_ADDR;
	_pp.value = DDC_START;
	ioctl(_ctrlFd, FIOREGSET, &_pp);
	usleep(100000);

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
			int ramAddr = i/8;
			int ramSelect = i%8 << 4;
			_pp.value = ddcSelect | DDC_STOP | ramSelect | ramAddr;
			_pp.offset = KAISER_ADDR;
			ioctl(_ctrlFd, FIOREGSET, &_pp);

			// write the value
			// LS word first
			_pp.value = kaiser[i] & 0xFFFF;
			_pp.offset = KAISER_DATA_LSW;
			ioctl(_ctrlFd, FIOREGSET, &_pp);
			// then the MS word -- since coefficients are 18 bits and FPGA registers are 16 bits!
			_pp.value = (kaiser[i] >> 16) & 0x3;
			_pp.offset = KAISER_DATA_MSW;
			ioctl(_ctrlFd, FIOREGSET, &_pp);

			// latch coefficient
			_pp.value = 0x1;
			_pp.offset = KAISER_WR;
			ioctl(_ctrlFd, FIOREGSET, &_pp);

			// disable writing (kaiser readback only succeeds if we do this)
			_pp.value = 0x0;
			_pp.offset = KAISER_WR;
			ioctl(_ctrlFd, FIOREGSET, &_pp);

			// read back the programmed value; we need to do this in two words as above.
			_pp.offset = KAISER_READ_LSW;
			ioctl(_ctrlFd, FIOREGGET, &_pp);
			readBack = _pp.value;
			_pp.offset = KAISER_READ_MSW;
			ioctl(_ctrlFd, FIOREGGET, &_pp);
			readBack |= (_pp.value << 16);
			if (readBack != kaiser[i]) {
				std::cout << "kaiser readback failed for coefficient "
						<< std::dec << i << std::hex << ", wrote " << kaiser[i]
						<< ", read " << readBack << std::endl;

				kaiserLoaded = false;
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
		for (unsigned int i = 0; i< gaussian.size(); i++) {

			unsigned int readBack;
			int ramAddr = i%8;
			int ramSelect = i/8 << 4;
			/// @todo early versions of the gaussian filter programming required
			/// the ds select bits to be set in the gaussian address register.
			/// We can take this out when we get a working bitstream with this
			/// fixed
			_pp.value = ddcSelect | ramSelect | ramAddr;
			_pp.offset = GUASSIAN_ADDR;

			// set the address
			ioctl(_ctrlFd, FIOREGSET, &_pp);

			// write the value
			// LS word first
			_pp.value = gaussian[i] & 0xFFFF;
			_pp.offset = GUASSIAN_DATA_LSW;
			ioctl(_ctrlFd, FIOREGSET, &_pp);
			// then the MS word -- since coefficients are 18 bits and FPGA registers are 16 bits!
			_pp.value = (gaussian[i] >> 16) & 0x3;
			_pp.offset = GUASSIAN_DATA_MSW;
			ioctl(_ctrlFd, FIOREGSET, &_pp);

			// enable writing
			_pp.value = 0x1;
			_pp.offset = GUASSIAN_WR;
			ioctl(_ctrlFd, FIOREGSET, &_pp);

			// disable writing (gaussian readback only succeeds if we do this)
			_pp.value = 0x0;
			_pp.offset = GUASSIAN_WR;
			ioctl(_ctrlFd, FIOREGSET, &_pp);

			// read back the programmed value; we need to do this in two words as above.
			_pp.offset = GUASSIAN_READ_LSW;
			ioctl(_ctrlFd, FIOREGGET, &_pp);
			readBack = _pp.value;
			_pp.offset = GUASSIAN_READ_MSW;
			ioctl(_ctrlFd, FIOREGGET, &_pp);
			readBack |= _pp.value << 16;
			if (readBack != gaussian[i]) {
				std::cout << "gaussian readback failed for coefficient "
						<< std::dec << i << std::hex << ", wrote "
						<< gaussian[i] << ", read " << readBack << std::endl;

				gaussianLoaded = false;
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


    // get the gaussian filter coeeficients.
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
		BuiltinGaussian builtins;
	    // The pulsewidth in microseconds. It must match one of those
	    // available in BuiltinGaussian.
	    double pulseWidthUs = 1.00;

		// Choose the correct builtin Gaussian filter coefficient set.
		switch (_pulse_width) {  // pulse width in 10 MHz counts
		case 2:
			pulseWidthUs = 0.2;
			break;
		case 4:
			pulseWidthUs = 0.4;
			break;
		case 6:
			pulseWidthUs = 0.6;
			break;
		case 8:
			pulseWidthUs = 0.8;
			break;
		case 10:
			pulseWidthUs = 1.0;
			break;
		case 12:
			pulseWidthUs = 1.2;
			break;
		case 14:
			pulseWidthUs = 1.4;
			break;
		case 16:
			pulseWidthUs = 1.6;
			break;
		default:
			std::cerr << "chip width specification of " << _pulse_width
					<< " is not recognized, filter will be configured for a "
					<< pulseWidthUs << " uS pulse\n";
			break;
		}
		if (builtins.find(pulseWidthUs) == builtins.end()) {
			std::cerr << "No entry for " << pulseWidthUs <<
				" us pulsewidth in list of builtin Gaussian filters!" <<
				std::endl;
			abort();
		}
		gaussian = FilterSpec(builtins[pulseWidthUs]);
	    std::cout << "Gaussian filter programmed for a " << pulseWidthUs << " uS pulse\n";
	}

	// get the kaiser filter coefficients
	FilterSpec kaiser;
	if (_kaiserFile.size() != 0) {
		FilterSpec k(_kaiserFile);
		if (!k.ok()) {
			std::cerr << "Incorrect or unaccesible filter definition: "
					<< _kaiserFile << std::endl;
			return -1;
		} else {
			kaiser = k;
		}
	} else {
		BuiltinKaiser k;
		kaiser = FilterSpec(k[5.0]); // select 5 MHz bandwidth filter
	}

	std::cout << "Kaiser filter programmed for " << 5.0 << " MHz bandwidth\n";

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

	double prtClock = (60e6); // Timer Input Clock Freq
	int periodCount; // Period Count for all Timers
	int PrtScheme; // PRT Scheme for all Timers

	// Internal Timing Setup

	unsigned int ALL_TIMERS= TIMER0 | TIMER1 | TIMER2 | TIMER3;

	// Calculate the period and PRT Scheme for dual prt or single prt
	int X, Y;
	if (_radarParams.radd_nipp == 2) //dual prt
	{
		periodCount = (int) (_radarParams.radd_ipp1
				* ((float) _radarParams.radd_ipp2 / _radarParams.radd_ipp1
						- (int) (_radarParams.radd_ipp2
								/ _radarParams.radd_ipp1))
				/ (int) (_radarParams.radd_ipp2 / _radarParams.radd_ipp1)
				* prtClock / 1e3);

		X = (int) ((int) (_radarParams.radd_ipp2 / _radarParams.radd_ipp1)
				/ ((float) _radarParams.radd_ipp2 / _radarParams.radd_ipp1
						- (int) (_radarParams.radd_ipp2
								/ _radarParams.radd_ipp1)));
		Y = (int) (X * _radarParams.radd_ipp2 / _radarParams.radd_ipp1);

		PrtScheme = (Y<<4) | X;
	} else //single prt
	{
		periodCount = (int) ceil((_radarParams.radd_ipp1 * prtClock / 1e3));
		PrtScheme = 0x0000;
	}

	Adapter_Write32(&_chanAdapter, V4, MT_ADDR, CONTROL_REG | ALL_TIMERS); // Control Register
	Adapter_Write32(&_chanAdapter, V4, MT_DATA, TIMER_ON); // Enable Timer
	Adapter_Write32(&_chanAdapter, V4, MT_WR, WRITE_ON); // Turn on Write Strobes

	// TIMER 0
	// Delay Register
	Adapter_Write32(&_chanAdapter, V4, MT_ADDR, DELAY_REG | TIMER0); // Address
	Adapter_Write32(&_chanAdapter, V4, MT_DATA, _radarParams.wave_chpoff[0]); // Data
	// Pulse Width Register
	Adapter_Write32(&_chanAdapter, V4, MT_ADDR, WIDTH_REG | TIMER0); // Address
	Adapter_Write32(&_chanAdapter, V4, MT_DATA, _radarParams.wave_chpwid[0]
			* _radarParams.wave_ngates[0]); // Data
	// TIMER 1
	// Delay Register
	Adapter_Write32(&_chanAdapter, V4, MT_ADDR, DELAY_REG | TIMER1); // Address
	Adapter_Write32(&_chanAdapter, V4, MT_DATA, _radarParams.wave_chpoff[1]); // Data
	// Pulse Width Register
	Adapter_Write32(&_chanAdapter, V4, MT_ADDR, WIDTH_REG | TIMER1); // Address
	Adapter_Write32(&_chanAdapter, V4, MT_DATA, _radarParams.wave_chpwid[1]
			* _radarParams.wave_ngates[1]); // Data
	// TIMER 2
	// Delay Register
	Adapter_Write32(&_chanAdapter, V4, MT_ADDR, DELAY_REG | TIMER2); // Address
	Adapter_Write32(&_chanAdapter, V4, MT_DATA, _radarParams.wave_chpoff[2]); // Data
	// Pulse Width Register
	Adapter_Write32(&_chanAdapter, V4, MT_ADDR, WIDTH_REG | TIMER2); // Address
	Adapter_Write32(&_chanAdapter, V4, MT_DATA, _radarParams.wave_chpwid[2]
			* _radarParams.wave_ngates[2]); // Data
	// TIMER 3
	// Delay Register
	Adapter_Write32(&_chanAdapter, V4, MT_ADDR, DELAY_REG | TIMER3); // Address
	Adapter_Write32(&_chanAdapter, V4, MT_DATA, _radarParams.wave_chpoff[3]); // Data
	// Pulse Width Register
	Adapter_Write32(&_chanAdapter, V4, MT_ADDR, WIDTH_REG | TIMER3); // Address
	Adapter_Write32(&_chanAdapter, V4, MT_DATA, _radarParams.wave_chpwid[3]
			* _radarParams.wave_ngates[3]); // Data
	// ALL TIMERS
	// Period Register
	Adapter_Write32(&_chanAdapter, V4, MT_ADDR, PERIOD_REG | ALL_TIMERS); // Address
	Adapter_Write32(&_chanAdapter, V4, MT_DATA, periodCount); // Data
	//Multiple PRT Register
	Adapter_Write32(&_chanAdapter, V4, MT_ADDR, PRT_REG | ALL_TIMERS); // Address
	Adapter_Write32(&_chanAdapter, V4, MT_DATA, PrtScheme); // Mult PRT Value Timer 0

	// Enable and Trigger All Timers
	Adapter_Write32(&_chanAdapter, V4,
	MT_ADDR,
	PRT_REG | ALL_TIMERS | TIMER_EN); // Set Global Enable

	usleep(1000);

	Adapter_Write32(&_chanAdapter, V4, MT_WR, WRITE_OFF); // Turn off Write Strobes
#endif
	return true;

}

/////////////////////////////////////////////////////////////////////////

void p7142hcrdn::startInternalTimer() {
#ifdef USE_TIMER

	//
	//    This start the internal timers.

	unsigned int ALL_TIMERS= TIMER0 | TIMER1 | TIMER2 | TIMER3;

	Adapter_Write32(&_chanAdapter, V4, MT_WR, WRITE_ON); // Turn on Write Strobes

	// Enable and Trigger All Timers
	Adapter_Write32(&_chanAdapter, V4,
	MT_ADDR,
	PRT_REG | ALL_TIMERS | ADDR_TRIG); // Set Global Enable

	usleep(1000);

	Adapter_Write32(&_chanAdapter, V4, MT_WR, WRITE_OFF); // Turn off Write Strobes

	// Get current system time as xmit start time
	setXmitStartTime(microsec_clock::universal_time());
#endif
}

//////////////////////////////////////////////////////////////////////
#ifdef TIME
void p7142hcrdn::setXmitStartTime(ptime startTime) {
	_xmitStartTime = startTime;
}
#endif

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

