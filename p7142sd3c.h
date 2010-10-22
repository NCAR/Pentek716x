/*
 * p7172sd3c.h
 *
 *  Created on: Jan 26, 2009
 *      Author: sd3c
 */

#ifndef P7172SD3C_H_
#define P7172SD3C_H_

#include "p7142.h"
#include "p7142sd3cDn.h"
#include <sys/types.h>
#include <sys/stat.h>

#include <string>
#include <vector>
#include <map>

#include <stdio.h>
#include <stdlib.h>

#include <boost/date_time/posix_time/posix_time.hpp>

namespace Pentek {

/// A p7142 class specialized for cards running the SD3C firmware.
///
/// <h2>The SD3C firmware</h2>
/// The Pentek 7142 and the associated SD3C firmware support four 
/// receiver channels. The first pair of channels share receive timing 
/// characteristics, as does the second pair.
///
/// <h2>Simulation</h2>
/// For development and testing without the transceiver hardware, the p7142sd3c can
/// be configured to operate in simulation mode. In this case, the p7142Dn class
/// is configured for simulation as well, and its read() function will synthesize
/// simulated data. p7142sd3c will add sync words and tags as appropriate, depending on the
/// operating mode. Synchronization errors are randomly inserted when operating in
/// simulation mode.
///
class p7142sd3c : public p7142 {
public:
    /// Constructor.
    /// @param devName The top level device name (e.g.,
    /// /dev/pentek/p7142/0. Use ok() to verify successful construction.
    /// @param simulate Set true for simulation mode.
    /// @param tx_delay the delay for the tx pulse in seconds
    /// @param prt The radar PRT in seconds
    /// @param prt2 The second PRT of a staggered PRT sequence in seconds
    /// @param staggeredPrt set true to use the second PRT for staggered PRT mode
    /// @param freeRun If true, the firmware will be configured to ignore the 
    ///     PRT gating.
    p7142sd3c(std::string devName, bool simulate, double tx_delay, 
        double tx_pulsewidth, double prt, double prt2, bool staggeredPrt, 
        bool freeRun);
    
    /// Destructor.
    virtual ~p7142sd3c();
    
    /// Construct and add a downconverter for one of our receiver channels.
    /// @param chanId The channel identifier (should be a zero based small integer)
    /// @param gates The number of gates
    /// @param nsum The number of coherent integrator sums. If < 2, the coherent integrator will not be used.
    /// @param tsLength The number of pulses in one time series. Used to set interrupt buffer size, so
    /// that we have reasonable responsiveness in the data stream.
    /// @param rx_delay the delay to the first rx gate in seconds
    /// @param pulse_width The radar pulse width in seconds
    /// @param gaussianFile Name of the file containing the Gaussian
    ///   filter parameters
    /// @param kaiserFile Name of the file containing the Kaiser
    ///   filter parameters
    /// @param simulate Set true if we operate in simulation mode.
    /// @param simPauseMS The number of milliseconds to wait between beams
    /// simulated data when calling read()
    /// @param simWaveLength The wavelength of the simulated data, in sample counts
    virtual p7142sd3cDn * addDownconverter(
            int chanId, 
            int gates, 
            int nsum,
            int tsLength,
            double rx_delay, 
            double rx_pulse_width,
            std::string gaussianFile, 
            std::string kaiserFile,
            double simPauseMS = 0.1,
            int simWaveLength = 5000,
            bool internalClock = false);
    
    /// @return The ADC clock frequency in Hz.
    double adcFrequency() const {
        return _adc_clock;
    }
    
    /// Convert a time in seconds to integer timer counts, which are in units
    /// of (2 / _adc_clock).
    /// @ param time the time to be converted, in seconds
    /// @ return the time in integer timer counts, which are in units of
    /// (2 / _adc_clock) seconds.
    int timeToCounts(double time) const;
    
    /// Convert a time in (2 / _adc_clock) integer timer counts to
    /// a time in seconds.
    /// @ param time the time to be converted, in (2 / _adc_clock) counts.
    /// @ return the time in seconds.
    double countsToTime(int counts) const;
    
    /// Control the timers.
    /// @param start Set true to start, set false to stop.
    void timersStartStop(bool start);
    
    /// @return Time of first transmit pulse
    boost::posix_time::ptime xmitStartTime() const {
        return _xmitStartTime;
    }

    /// @return The first PRT, in seconds
    double prt() const {
        return countsToTime(_prtCounts);
    }
    
    /// @return The first PRT, in units of (2 / adcFrequency())
    unsigned int prtCounts() const {
        return _prtCounts;
    }
    
    /// @return The second PRT, in seconds, or zero if not running staggered
    /// PRT.
    double prt2() const {
        return countsToTime(_prt2Counts);
    }
    
    /// @return The second PRT, in units of (2 / adcFrequency()), or
    ///     zero if not running staggered PRT.
    unsigned int prt2Counts() const {
        return(_staggeredPrt ? _prt2Counts : 0);
    }
    
    /// @return The FPGA firmware software repository revision number.
    int fpgaRepoRevision();

    /// Set the time of the first transmit pulse.
    /// @param startTime The boost::posix_time::ptime of the first transmit
    ///    pulse.
    void setXmitStartTime(boost::posix_time::ptime startTime) {
        _xmitStartTime = startTime;
    }
    /// Read the ttl input lines from the fpga
    /// @return The input line values.
    unsigned short int TTLIn();

    /// Set the ttl output lines on the FPGA.
    /// @param data The value to be written.
    void TTLOut(unsigned short int data);

    /// The types of downconverters that can be instantiated in the SD3C Pentek 
    /// firmware.
    typedef enum {
        DDC10DECIMATE, DDC8DECIMATE, DDC4DECIMATE, BURST
    } DDCDECIMATETYPE;

    /// Return our DDC type
    /// @return the DDC type instantiated in our card's firmware
    DDCDECIMATETYPE ddcType() { return _ddcType; }
    
    /// Return the name of the firmware DDC type
    /// @return the name of the firmware DDC type
    std::string ddcTypeName() const { return ddcTypeName(_ddcType); }
    
    /// Return the name of the given DDCDECIMATETYPE
    static std::string ddcTypeName(DDCDECIMATETYPE type) {
        switch (type) {
        case DDC10DECIMATE:
            return std::string("DDC10DECIMATE");
        case DDC8DECIMATE:
            return std::string("DDC8DECIMATE");
        case DDC4DECIMATE:
            return std::string("DDC4DECIMATE");
        case BURST:
            return std::string("BURST");
        default:
            return std::string("Unknown");
        }
    }
    
    /// Set the filter start bit, which starts the data flow for all channels.
    void startFilters();

    /// Stop the filters
    void stopFilters();
    
    /// The transmit pulse width, in seconds
    /// @return the transmit pulse width, in seconds
    double txPulseWidth() const { return countsToTime(txPulseWidthCounts()); }
    
    /// Return the transmit pulse width, in local counts, which are units of 
    /// (2 / adc_freq) seconds.
    int txPulseWidthCounts() const { return _timerWidth(TX_PULSE_TIMER); }
    
    /// Set up general purpose timer 0. This timer is not used internally by
    /// SD3C, but is made available on an external pin.
    /// @param delay the delay for the timer, in seconds
    /// @param width the width for the timer pulse, in seconds
    void setGPTimer0(double delay, double width) {
        _setTimer(GP_TIMER_0, timeToCounts(delay), timeToCounts(width));
    }
    
    /// Set up general purpose timer 1. This timer is not used internally by
    /// SD3C, but is made available on an external pin.
    /// @param delay the delay for the timer, in seconds
    /// @param width the width for the timer pulse, in seconds
    void setGPTimer1(double delay, double width) {
        _setTimer(GP_TIMER_1, timeToCounts(delay), timeToCounts(width));
    }
    
    /// Set up general purpose timer 2. This timer is not used internally by
    /// SD3C, but is made available on an external pin.
    /// @param delay the delay for the timer, in seconds
    /// @param width the width for the timer pulse, in seconds
    void setGPTimer2(double delay, double width) {
        _setTimer(GP_TIMER_2, timeToCounts(delay), timeToCounts(width));
    }
    
    /// Set up general purpose timer 3. This timer is not used internally by
    /// SD3C, but is made available on an external pin.
    /// @param delay the delay for the timer, in seconds
    /// @param width the width for the timer pulse, in seconds
    void setGPTimer3(double delay, double width) {
        _setTimer(GP_TIMER_3, timeToCounts(delay), timeToCounts(width));
    }
    
    /// Set the DDC type when simulating a P7142 card.
    static void setSimulateDDCType(DDCDECIMATETYPE type) {
        _simulateDDCType = type;
    }
    
    friend class p7142sd3cDn;
    
protected:
    /**
     * SD3C has dedicated uses for the first four of its eight timers. Here 
     * we provide convenient names which map to the indices of the timers.
     *  (0) MASTER_SYNC_TIMER - This timer provides the trigger which starts the 
     *      process of transmitting and receiving a pulse. Delays in other timers
     *      are relative to this trigger.
     *  (1) RX_01_TIMER - While this timer is on, data are sampled for channels 0 and 1
     *  (2) TX_PULSE_TIMER - The transmitter fires while this timer is on
     *  (3) GP_TIMER_0 - This timer is not used internally, but is routed
     *      to an external pin. For HCR, profiler, and Ka, this timer is used
     *      for modulation of the transmit pulse.
     *  (4) RX_23_TIMER - While this timer is on, data are sampled for channels 2 and 3
     *  (5) GP_TIMER_1 - This timer is not used internally, but is routed
     *      to an external pin.
     *  (6) GP_TIMER_2 - This timer is not used internally, but is routed
     *      to an external pin.
     *  (7) GP_TIMER_3 - This timer is not used internally, but is routed
     *      to an external pin.
     */
    typedef enum {
        MASTER_SYNC_TIMER, // timer 0 is the master sync timer
        RX_01_TIMER,       // timer 1 is the rx timer for channels 0 and 1
        TX_PULSE_TIMER,    // timer 2 is the tx pulse timer
        GP_TIMER_0,        // timer 3 is not used internally, but is routed
                           // externally for general purpose use
        RX_23_TIMER,       // timer 4 is the rx timer for channels 2 and 3
        GP_TIMER_1,        // timer 5 is not used internally, but is routed
                           // externally for general purpose use
        GP_TIMER_2,        // timer 6 is not used internally, but is routed
                           // externally for general purpose use
        GP_TIMER_3,        // timer 7 is not used internally, but is routed
                           // externally for general purpose use
        N_SD3C_TIMERS      // The count of SD3C timers, i.e., 8
    } TimerIndex;
    
    /**
     * ID bits associated with the eight SD3C timers; these are used for
     * ioctl-s dealing with the timers.
     */
    static const unsigned int SD3C_TIMER_BITS[N_SD3C_TIMERS];
    /**
     * ALL_SD3C_TIMER_BITS is a bit mask for operations on all eight timers.
     */
    static const unsigned int ALL_SD3C_TIMER_BITS;

    /**
     * Return timer delay in counts for the selected timer. While
     * an integer index may be used explicitly, it is recommended to use
     * a TimerIndex enumerated value instead.
     * @param timerNdx the integer (or TimerIndex) index for the timer of
     *     interest
     * @return the timer delay in counts
     */
    int _timerDelay(int timerNdx) const {
        return(_timers[timerNdx].delay());
    }
    
    /**
     * Return timer width in counts for the selected timer. While
     * an integer index may be used explicitly, it is recommended to use
     * a TimerIndex enumerated value instead.
     * @param timerNdx the integer (or TimerIndex) index for the timer of
     *     interest
     * @return the timer width in counts
     */
    int _timerWidth(int timerNdx) const {
        return(_timers[timerNdx].width());
    }
    
    /// Reset the digital clock managers on the FPGA. Necessary since
    /// some of the new DCMs we have added in the firmware use the
    /// CLKFX output, which won't lock at startup.
    void _resetDCM();

    /**
     * Set delay and width values for the selected timer. Note that values
     * set here are not actually loaded onto the card until the timers are 
     * started with timersStartStop().
     * 
     * @param timerNdx the TimerIndex for the timer to be set
     * @param delay the delay in counts for the timer
     * @param width the width in counts for the timer to be held on
     */
    void _setTimer(TimerIndex ndx, int delay, int width, bool verbose = true);
    
    /// Load configured timer values onto the device.
    /// @return true if successful, false otherwise.
    bool _initTimers();
    
    /**
     * Load the device's FREERUN register.
     */
    void _loadFreeRun();

    /// @return Read the type of DDC instantiated in the firmware
    DDCDECIMATETYPE _readDDCType();

    /**
     * Simple class to hold integer delay and width for a timer.
     */
    class _DelayAndWidth {
    public:
        _DelayAndWidth(int delay, int width) : _delay(delay), _width(width) {}
        _DelayAndWidth() : _delay(0), _width(0) {}
        int delay() const { return _delay; }
        int width() const { return _width; }
    private:
        int _delay;
        int _width;
    };
    
    /**
     * vector of delay/width pairs for our 8 SD3C timers
     */
    _DelayAndWidth _timers[N_SD3C_TIMERS];
    /// radar PRT in _adc_clock/2 counts
    unsigned int _prtCounts;
    /// second PRT of staggered PRT in _adc_clock/2 counts
    unsigned int _prt2Counts;
    /// Staggered PRT flag. If true, both PRT values are used in staggered
    /// mode.
    bool _staggeredPrt;
    /// Free-run mode flag. If true, the firmware is be configured to ignore PRT 
    /// gating.
    bool _freeRun;
    /// Time of the first xmit pulse.
    boost::posix_time::ptime _xmitStartTime;
    /// peek-poke structure pointer
    ARG_PEEKPOKE _pp;
    /// The adc clock rate in Hz
    double _adc_clock;
    /// The prf(s) in Hz
    double _prf;
    /// The dual prf in Hz.
    double _prf2;
    /// DDC type instantiated in our card's firmware
    DDCDECIMATETYPE _ddcType;
    /// DDC type to use when simulating (default DDC8DECIMATE).
    static DDCDECIMATETYPE _simulateDDCType;
};

}

#endif /* P7172SD3C_H_ */
