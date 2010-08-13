/*
 * p7172sd3c.h
 *
 *  Created on: Jan 26, 2009
 *      Author: sd3c
 */

#ifndef P7172SD3C_H_
#define P7172SD3C_H_

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

/// A p7142 downconvertor adapted for specific use with the SD3C firmware.
///
/// <b>The SD3C firmware</b>
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
class p7142sd3cdn: public p7142dn {
public:
	/// The type of downconverter that is instantiated in the pentek firmware.
	enum DDCDECIMATETYPE {
		DDC8DECIMATE, DDC4DECIMATE
	};
	
	/// The SD3C synchronization word value.
	static const uint32_t SD3C_SYNCWORD = 0xAAAAAAAA;

	/// Collects timer initialization values
	struct TimerSetup {
		/// The timer id (e.g. TIMER0)
		int id;
		/// The delay, timer clock counts 
		int delay;
		/// The width, in timer clock counts
		int width;
		TimerSetup(int Id, int Delay, int Width):
			id(Id), delay(Delay), width(Width){}
	};

	
    /// Constructor.
    /// All times are expressed in seconds.
    /// @param devName The top level device name (e.g.
    /// /dev/pentek/p7140/0.
    /// @param chanId The channel identifier (should be a zero based small integer)
    /// @param gates The number of gates
    /// @param nsum The number of coherent integrator sums. If < 2, the coherent integrator will not be used.
    /// @param tsLength The number of pulses in one time series. Used to set interrupt buffer size, so
    /// that we have reasonable responsiveness in the data stream.
    /// @param rx_delay the delay to the first rx gate in seconds
    /// @param tx_delay the delay for the tx pulse in seconds
    /// @param prt The radar PRT in seconds
    /// @param prt2 The second PRT of a staggered PRT sequence in seconds
    /// @param pulse_width The radar pulse width in seconds
    /// @param stgr_prt the staggered PRT flag; 1 = stagger, 0 = uniform
    /// @param timer_delays A vector containing delays for the 5 general purpose times, in seconds
    /// @param timer_widths A vector containing widths for the 5 general purpose times, in seconds
    /// @param freeRun If true, the firmware will be configured to ignore the PRT gating.
    /// @param gaussianFile Name of the file containing the Gaussian
    ///   filter parameters
    /// @param kaiserFile Name of the file containing the Kaiser
    ///   filter parameters
    /// @param simulate Set true if we operate in simulation mode.
    /// @param simPauseMS The number of milliseconds to wait before returning
    /// simulated data when calling read();
    /// @param internalClock Set true if the internal clock should be
    /// used instead of the front panel clock.
    p7142sd3cdn(
        std::string devName, 
        int chanId, 
        int gates, 
        int nsum,
        int tsLength,
        double rx_delay, 
        double tx_delay, 
        double prt, 
        double prt2, 
        double pulse_width,
        bool stgr_prt, 
        std::vector<double> timer_delays,
        std::vector<double> timer_widths,
        bool freeRun, 
        std::string gaussianFile, 
        std::string kaiserFile,
        bool simulate = false, 
        int simPauseMS = 100, 
        bool internalClock = false);
    
	/// Destructor
	virtual ~p7142sd3cdn();

    /// Read bytes. If in simulated mode, a sine wave with wavelength
    /// of _simWaveLength gates will be synthesized. It will have some 
    /// random noise applied as well.
    /// @param buf read bytes into this buffer
    /// @param bufsize The number of bytes tor read.
    /// @return The actual number of bytes read
    virtual int read(char* buf, int bufsize);
    
	/// @return The FPGA firmware software repository revision number.
	int fpgaRepoRevision();

	/// @return The type of DDC instantiated in the firmware
	DDCDECIMATETYPE ddc_type();

	/// Set the filter start bit, which starts the data flow. Applies only to channel 0
	/// @todo Fix the start logic - really does not belong per channel
	void startFilters();

	/// Stop the filters
	void stopFilters();

	/// Control the timers.
	/// @param start Set true to start, set false to stop.
	void timersStartStop(bool start);
	
    /// @return Time of first transmit pulse
    boost::posix_time::ptime xmitStartTime() const {
        return _xmitStartTime;
    }
    
    /// @return The first PRT, in units of (2 / adcFrequency())
    int prt() const {
        return _prt;
    }
    
    /// @return The second PRT, in units of (2 / adcFrequency()), or
    ///     zero if not running staggered PRT.
    int prt2() const {
        return(_staggeredPrt ? _prt2 : 0);
    }
    

    /// @return The receiver pulsewidth, in s
    double rcvrPulseWidth() const {
        return(_timer_widths[2] / (_adc_clock / 2));
    }
    
    /// @return The receiver delay to first gate, in s
    double rcvrFirstGateDelay() const {
        return(_timer_delays[1] / (_adc_clock / 2));
    }
    
    static const double SPEED_OF_LIGHT = 2.99792458e8;  // m s-1
    
    /// @return The gate spacing, in m
    double gateSpacing() const {
        return(0.5 * SPEED_OF_LIGHT * rcvrPulseWidth());
    }
    
    /// @return The range to the leading edge of the first gate, in m
    double rangeToFirstGate() const {
        return(0.5 * SPEED_OF_LIGHT * rcvrFirstGateDelay());
    }
    
    /// @return Time of the given transmit pulse.
    boost::posix_time::ptime timeOfPulse(unsigned long pulseNum) const;
    
	/// @return The data bandwidth in bytes per second
	int dataRate();
	
	/// @return The ADC clock frequency in Hz.
	double adcFrequency() const {
	    return _adc_clock;
	}

protected:
	/// Configure the p7142sd3cdn
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
	bool initTimers();

	/// Set the time of the first transmit pulse.
	/// @param startTime The boost::posix_time::ptime of the first transmit
	///    pulse.
	void setXmitStartTime(boost::posix_time::ptime startTime);

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

	/// Set the free run control bit in the transceiver
	/// control register, as specified by _freeRun
	void freeRunConfig();

	/// Perform reset functions:
	///
	/// - clear the fifos
	void reset();
	
	/// Search through the given buffer and return the index of the next
	/// occurrence of SD3C_SYNCWORD. If the sync word is not found, return
	/// -1.
	/// @param buf the byte buffer to search
	/// @param buflen the number of bytes in the buffer
	/// @return the index of the next occurrence of SD3C_SYNCWORD, or
	/// -1 if the sync word is not found.
	static int indexOfNextSync(const char* buf, int buflen);

	/// number of gates
	int _gates;
	/// number of coherent integrator sums
	int _nsum;
	//// The time series length
	int _tsLength;
	/// radar PRT in _adc_clock/2 counts
	int _prt;
	/// second PRT of staggered PRT in _adc_clock/2 counts
	int _prt2;
	/// radar pulse width in _adc_clock/2 counts
	//int _pulseWidth;
    /// receiver delay to first gate in _adc_clock/2 counts
    //int _rx_delay;
    /// transmit pulse delay  in _adc_clock/2 counts
    //int _tx_delay;
	/// staggered PRT flag: 1 = stagger, 0 = uniform
	bool _staggeredPrt;
	/// free running mode. Causes the firmware to be configured to ignore the PRT gating.
	bool _freeRun;
	/// The type of downconverter instantiated in the firmware
	DDCDECIMATETYPE _ddcType;
	/// The path to the file containing the gaussian filter definitions.
	std::string _gaussianFile;
	/// The path to the file containing the kaiser filter coefficients.
	std::string _kaiserFile;
    /// The pulse number if we're simulating data
    int _simPulseNum;
	/// Time of the first xmit pulse.
	boost::posix_time::ptime _xmitStartTime;
	/// peek-poke structure pointer
	ARG_PEEKPOKE _pp;
	/// The adc clock rate in Hz
	double _adc_clock;
	/// The prf(s) in Hz
	double _prf;
    double _prf2;
    /// The timer delay counts, in adc_clock/2 counts, for all 8 timers. The first
    /// three entries are derived calculated in this class, the remaining 
    /// 5 are specified by the caller via the 'delays' parameter in the 
    /// constructor.
    std::vector<int> _timer_delays;
    /// The timer width counts, in adc_clock/2 counts, for all 8 timers. The first
    /// three entries are derived calculated in this class, the remaining 
    /// 5 are specified by the caller via the 'widths' parameter in the 
    /// constructor.
    std::vector<int> _timer_widths;
private:
    /// Open the control device.
    void _openControlDevice();

};

}

#endif /* P7172SD3C_H_ */
