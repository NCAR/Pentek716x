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
		int tsLength, int delay, int prt, int prt2, int pulse_width,
		bool stgr_prt, std::string gaussianFile, std::string kaiserFile,
		DDCDECIMATETYPE decimateType, int bypdivrate, bool simulate,
		int simPauseMS, bool internalClock) :
	p7142dn(devName, chanId, bypdivrate, simulate, simPauseMS, internalClock),
			_gates(gates), _nsum(nsum), _tsLength(tsLength), _delay(delay),
			_prt(prt), _prt2(prt2), _pulse_width(pulse_width), _stgr_prt(
					stgr_prt), _gaussianFile(gaussianFile), _kaiserFile(
					kaiserFile), _decimateType(decimateType)

{
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

	// how many bytes are there in each time series?
	int tsBlockSize;
	if (_nsum < 2) {
		tsBlockSize = tsLength * _gates * 2 * 2;
	} else {
		// coherently integrated data has:
		// 4 tags followed by even IQ pairs followed by odd IQ pairs,
		// for all gates. Tags, I and Q are 4 byte integers.
		tsBlockSize = tsLength * (4 + _gates * 2 * 2) * 4;
	}

	double pulseFreq = 1.0 / (prt / (10.0e6));
	double tsFreq = pulseFreq / tsLength;

	// we want the interrupt buffer size to be a multiple of tsBlockSize,
	// but no more than 20 interrupts per second.
	int intBlocks = 1;
	if (tsFreq <= 20) {
		intBlocks = 1;
	} else {
		intBlocks = (tsFreq / 20) + 1;
	}

	int bufferSize = tsBlockSize * intBlocks;

	std::cout << "prt is " << prt << "  prt frequency is " << pulseFreq
			<< "  ts freq is " << tsFreq << "  tsblocks per interrupt is "
			<< intBlocks << std::endl;

	std::cout << "pentek interrupt buffer size is " << bufferSize << std::endl;

	// set the buffer size
	bufset(_dnFd, bufferSize, 2);

	std::cout << "FPGA repository revision is " << fpgaRepoRevision()
			<< std::endl;

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

	// stop the filters if they are running.
	_pp.offset = KAISER_ADDR;
	ioctl(_ctrlFd, FIOREGGET, &_pp);
	_pp.offset = KAISER_ADDR;
	_pp.value = DDC_STOP;
	ioctl(_ctrlFd, FIOREGSET, &_pp);
	usleep(10000);
	_pp.offset = KAISER_ADDR;
	ioctl(_ctrlFd, FIOREGGET, &_pp);

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

	// Configure ADC FIFO Control for this channel


	_pp.offset = ppOffset;
	_pp.value = readBack & 0x000034BF;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	// set number of gates
	setGates(_gates);

	// set number of coherent integrator sums
	setNsum(_nsum);

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
	return _pp.value;

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
void p7142hcrdn::setGates(int gates) {
	_pp.offset = RADAR_GATES;
	_pp.value = gates;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	ioctl(_ctrlFd, FIOREGGET, &_pp);
	std::cout << "gates are " << _pp.value << std::endl;
}

//////////////////////////////////////////////////////////////////////
void p7142hcrdn::setNsum(int nsum) {
	_pp.offset = CI_NSUM;
	_pp.value = nsum;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	ioctl(_ctrlFd, FIOREGGET, &_pp);
	std::cout << "nsum is " << _pp.value << std::endl;
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

			int ramAddr = i / 8;
			int ramSelect = i % 8 << 4;
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
			int ramAddr = i % 8;
			int ramSelect = i / 8 << 4;
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
		// The pulsewidth in microseconds. It must match one of those
		// available in BuiltinGaussian.
		double pulseWidthUs = 1.00;
		gaussianFilterName = "ddc8_1_0";

		// Choose the correct builtin Gaussian filter coefficient set.
		switch (_decimateType) {
		case DDC8DECIMATE: {
			switch (_pulse_width) { // pulse width in 10 MHz counts
			case 2:
				pulseWidthUs = 0.2;
				gaussianFilterName = "ddc8_0_2";
				break;
			case 4:
				pulseWidthUs = 0.4;
				gaussianFilterName = "ddc8_0_4";
				break;
			case 6:
				pulseWidthUs = 0.6;
				gaussianFilterName = "ddc8_0_6";
				break;
			case 8:
				pulseWidthUs = 0.8;
				gaussianFilterName = "ddc8_0_8";
				break;
			case 10:
				pulseWidthUs = 1.0;
				gaussianFilterName = "ddc8_1_0";
				break;
			case 12:
				pulseWidthUs = 1.2;
				gaussianFilterName = "ddc8_1_2";
				break;
			case 14:
				pulseWidthUs = 1.4;
				gaussianFilterName = "ddc8_1_4";
				break;
			case 16:
				pulseWidthUs = 1.6;
				gaussianFilterName = "ddc8_1_6";
				break;
			default:
				std::cerr << "chip width specification of " << _pulse_width
						<< " is not recognized, filter will be configured for a "
						<< pulseWidthUs << " uS pulse\n";
				break;
			}
			break;
		}
		case DDC4DECIMATE: {
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
		std::cout << "Using gaussian filter coefficient set " << gaussianFilterName << std::endl;
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
		switch (_decimateType) {
		case DDC8DECIMATE: {
			kaiserFilterName = "ddc8_5_0";
			break;
		}
		case DDC4DECIMATE: {
			kaiserFilterName = "ddc4_5_0";
			break;
		}
		}
		if (builtins.find(kaiserFilterName) == builtins.end()) {
			std::cerr << "No entry for " << kaiserFilterName
					<< " in the list of builtin Kaiser filters!" << std::endl;
			abort();
		}
		kaiser = FilterSpec(builtins[kaiserFilterName]);
		std::cout << "Using kaiser filter coefficient set " << kaiserFilterName << std::endl;
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

	double prtClock = (62.5e6); // Timer Input Clock Freq
	int periodCount; // Period Count for all Timers
	int PrtScheme; // PRT Scheme for all Timers

	// Internal Timing Setup

	unsigned int ALL_TIMERS = TIMER0 | TIMER1 | TIMER2 | TIMER3 | TIMER4
			| TIMER5 | TIMER6 | TIMER7;

	// Calculate the period and PRT Scheme for dual prt or single prt
	int X, Y;
	float prt_ms, prt2_ms;
	if (_stgr_prt == true) //dual prt
	{
		prt_ms = (float) _prt / 1e3;
		prt2_ms = (float) _prt2 / 1e3;

		periodCount = (int) (prt_ms * (prt2_ms / prt_ms - (int) (prt2_ms
				/ prt_ms)) / (int) (prt2_ms / prt_ms) * prtClock / 1e3);

		X = (int) ((int) (prt2_ms / prt_ms) / (prt2_ms / prt_ms
				- (int) (prt2_ms / prt_ms)));
		Y = (int) (X * prt2_ms / prt_ms);

		PrtScheme = (Y << 4) | X;
	} else //single prt
	{
		periodCount = (int) ceil((_prt * prtClock / 10e6));
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
	_pp.value = (int) ceil((_delay * prtClock / 10e6));
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	// Pulse Width Register
	_pp.offset = MT_ADDR; // Address
	_pp.value = WIDTH_REG | TIMER0;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	_pp.offset = MT_DATA; // Data
	_pp.value = (int) ceil((_pulse_width * _gates * prtClock / 62.5e6));
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	// For now configure all 8 Timers identically, later we will customize per application
	// TIMER 1
	// Delay Register
	_pp.offset = MT_ADDR; // Address
	_pp.value = DELAY_REG | TIMER1;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	_pp.offset = MT_DATA; // Data
	_pp.value = (int) ceil((_delay * prtClock / 10e6));
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	// Pulse Width Register
	_pp.offset = MT_ADDR; // Address
	_pp.value = WIDTH_REG | TIMER1;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	_pp.offset = MT_DATA; // Data
	_pp.value = (int) ceil((_pulse_width * prtClock / 10e6));
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	// TIMER 2
	// Delay Register
	_pp.offset = MT_ADDR; // Address
	_pp.value = DELAY_REG | TIMER2;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	_pp.offset = MT_DATA; // Data
	_pp.value = (int) ceil((_delay * prtClock / 10e6));
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	// Pulse Width Register
	_pp.offset = MT_ADDR; // Address
	_pp.value = WIDTH_REG | TIMER2;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	_pp.offset = MT_DATA; // Data
	_pp.value = (int) ceil((_pulse_width * prtClock / 10e6));
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	// TIMER 3
	// Delay Register
	_pp.offset = MT_ADDR; // Address
	_pp.value = DELAY_REG | TIMER3;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	_pp.offset = MT_DATA; // Data
	_pp.value = (int) ceil((_delay * prtClock / 10e6));
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	// Pulse Width Register
	_pp.offset = MT_ADDR; // Address
	_pp.value = WIDTH_REG | TIMER3;
	ioctl(_ctrlFd, FIOREGSET, &_pp);
	_pp.offset = MT_DATA; // Data
	_pp.value = (int) ceil((_pulse_width * prtClock / 10e6));
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	// TIMER 4
	// Delay Register
	_pp.offset = MT_ADDR; // Address
	_pp.value = DELAY_REG | TIMER4;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	_pp.offset = MT_DATA; // Data
	_pp.value = (int) ceil((_delay * prtClock / 10e6));
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	// Pulse Width Register
	_pp.offset = MT_ADDR; // Address
	_pp.value = WIDTH_REG | TIMER4;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	_pp.offset = MT_DATA; // Data
	_pp.value = (int) ceil((_pulse_width * prtClock / 10e6));
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	// TIMER 5
	// Delay Register
	_pp.offset = MT_ADDR; // Address
	_pp.value = DELAY_REG | TIMER5;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	_pp.offset = MT_DATA; // Data
	_pp.value = (int) ceil((_delay * prtClock / 10e6));
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	// Pulse Width Register
	_pp.offset = MT_ADDR; // Address
	_pp.value = WIDTH_REG | TIMER5;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	_pp.offset = MT_DATA; // Data
	_pp.value = (int) ceil((_pulse_width * prtClock / 10e6));
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	// TIMER 6
	// Delay Register
	_pp.offset = MT_ADDR; // Address
	_pp.value = DELAY_REG | TIMER6;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	_pp.offset = MT_DATA; // Data
	_pp.value = (int) ceil((_delay * prtClock / 10e6));
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	// Pulse Width Register
	_pp.offset = MT_ADDR; // Address
	_pp.value = WIDTH_REG | TIMER6;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	_pp.offset = MT_DATA; // Data
	_pp.value = (int) ceil((_pulse_width * prtClock / 10e6));
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	// TIMER 7
	// Delay Register
	_pp.offset = MT_ADDR; // Address
	_pp.value = DELAY_REG | TIMER7;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	_pp.offset = MT_DATA; // Data
	_pp.value = (int) ceil((_delay * prtClock / 10e6));
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	// Pulse Width Register
	_pp.offset = MT_ADDR; // Address
	_pp.value = WIDTH_REG | TIMER7;
	ioctl(_ctrlFd, FIOREGSET, &_pp);

	_pp.offset = MT_DATA; // Data
	_pp.value = (int) ceil((_pulse_width * prtClock / 10e6));
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

	// Turn off Write Strobes
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

