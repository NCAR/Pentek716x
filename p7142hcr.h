/*
 * p7172hcr.h
 *
 *  Created on: Jan 26, 2009
 *      Author: hcr
 */

#ifndef P7172HCR_H_
#define P7172HCR_H_

#include "p7142.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <sys/types.h>
#include <sys/stat.h>
#include <pthread.h>

#include <string>
#include <vector>
#include <deque>
#include <map>

#include <stdio.h>
#include <stdlib.h>
#include "FilterSpec.h"

namespace Pentek {

/// A p7142 downconvertor adapted for specific use with the HCR firmware.
///
/// <b>The HCR firmware</b>
/// This firmware contains identical user blocks on all 4 ADC channels. The user blocks
/// contain downconvertors implemented with the csac filters. The csac filters
/// have a guassian and a kaiser filter in series. The filter coefficients are
/// configurable.
///
/// The downconvertors is followed by a coherent integrators. The coherent integrators
/// can by bypassed by specifying a coherent integration sum of 1.
/// In coherent integrator bypass mode, the data are delivered as packed 16 bit
/// I and Q values, along with beam tags. In coherent integration mode, the data
/// are delivered as 32 bit sums, for I and Q, for even and odd pulses. Beam tags
/// are also included in this mode.
class p7142hcrdn: public p7142dn {
public:
	///The type of downconverter that is instantiated in the pentek firmware.
	enum DDCDECIMATETYPE {
		DDC8DECIMATE, DDC4DECIMATE
	};

	/// Constructor
	/// @param devName The top level device name (e.g.
	/// /dev/pentek/p7140/0.
	/// @param chanId The channel identifier (should be a zero based small integer)
	/// @param gates The number of gates
	/// @param nsum The number of coherent integrator sums
	/// @param tsLength The number of pulses in one time series. Used to set interrupt buffer size, so
	/// that we have reasonable responsiveness in the data stream.
	/// @param delay the delay to the first gate in 10 MHz counts
	/// @param prt the radar PRT in 10 MHz counts
	/// @param prt2 the second PRT of a staggered PRT sequence, expressed in 10 MHz counts
	/// @param pulse_width the radar pulse width in 10 MHz counts
	/// @param stgr_prt the staggered PRT flag; 1 = stagger, 0 = uniform
	/// @param gaussianFile Name of the file containing the Gaussian
	///   filter parameters
	/// @param kaiserFile Name of the file containing the Kaiser
	///   filter parameters
	/// @param decimateType The type of downconverter instantiated in the pentek firmware
	/// @param bypassdivrate The bypass divider (decimation) rate
	/// @param simulate Set true if we operate in simulation mode.
	/// @param simPauseMS The number of milliseconds to wait before returning
	/// simulated data when calling read();
	/// @param internalClock Set true if the internal clock should be
	/// used instead of the front panel clock.
	p7142hcrdn(std::string devName, int chanId, int gates, int nsum,
			int tsLength, int delay, int prt, int prt2, int pulse_width,
			bool stgr_prt, std::string gaussianFile, std::string kaiserFile,
			DDCDECIMATETYPE ddcType, int decimation = 1, bool simulate =
					false, int simPauseMS = 100, bool internalClock = false);
	/// Destructor
	virtual ~p7142hcrdn();

	/// @return The FPGA firmware software repository revision number.
	int fpgaRepoRevision();

	/// @return The type of DDC instantiated in the firmware
	DDCDECIMATETYPE ddc_type();

protected:
	/// Set the filter start bit, which starts the data flow. Applies only to channel 0
	/// @todo Fix the start logic - really does not belong per channel
	void startFilters();

	/// Stop the filters
	void stopFilters();

	/// Configure the p7142hcrdn
	/// @return True if the configuration was successful
	bool config();

	/// Reset the digital clock managers on the FPGA. Necessary since
	/// some of the new DCMs we have added in the firmware use the
	/// CLKFX output, which won't lock at startup.
	void resetDCM(int fd);

	/// Configure the down converter fifos
	void fifoConfig();

	/// set the number of gates
	void setGates(int gates);

	/// set the number of sums
	void setNsum(int nsum);

	/// Configure the filters and the decimation value.
	int filterSetup();

	/// Program the cofficients for the gaussian and
	/// kaiser filters.
	bool loadFilters(FilterSpec& gaussian, FilterSpec& kaiser);

	/// Configure the timers.
	/// @return true if successful, false otherwise.
	bool timerInit();

	/// Starts the timers.
	void startInternalTimer();

	/// Set the time of the first transmit pulse.
	/// @param startTime The boost::posix_time::ptime of the first transmit
	///    pulse.
	void setXmitStartTime(boost::posix_time::ptime startTime);

	/// Time of first transmit pulse
	boost::posix_time::ptime xmitStartTime() {
		return _xmitStartTime;
	}

	/// Read the ttl input lines from the fpga
	/// @return The input line values.
	unsigned short int TTLIn();

	/// Set the ttl output lines on the FPGA.
	/// @param data The value to be written.
	void TTLOut(unsigned short int data);

	/// Set the interrupt buffer size for the pentek. Useful
	/// for trying to control the interrupt rate, based on how
	/// fast the FPGA is running.
	/// @todo We don't currently use this, because there were
	/// indications that it was causing the driver to drop
	/// blocks of data.
	void setInterruptBufSize();

	/// number of gates
	int _gates;
	/// number of coherent integrator sums
	int _nsum;
	//// The time series length
	int _tsLength;
	/// radar PRT in 10 MHz counts
	int _prt;
	/// second PRT of staggered PRT in 10 MHz counts
	int _prt2;
	/// radar pulse width in 10 MHz counts
	int _pulseWidth;
	/// delay to first gate in 10 MHz counts
	int _delay;
	/// staggered PRT flag: 1 = stagger, 0 = uniform
	bool _staggeredPrt;
	/// The type of downconverter instantiated in the firmware
	DDCDECIMATETYPE _ddcType;
	/// The path to the file containing the gaussian filter definitions.
	std::string _gaussianFile;
	/// The path to the file containing the kaiser filter coefficients.
	std::string _kaiserFile;
	/// Time of the first xmit pulse.
	boost::posix_time::ptime _xmitStartTime;
	/// peak-poke structure pointer
	ARG_PEEKPOKE _pp;

};

}

#endif /* P7172HCR_H_ */
