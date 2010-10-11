// -*- mode: C++; indent-tabs-mode: nil; c-basic-offset: 2; tab-width: 2; -*-
// vim: set shiftwidth=2 softtabstop=2 expandtab:
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

  /// A p7142 class adapted for specific use with the downconversion
  /// function of the SD3C firmware.
  ///
  /// <h2>The SD3C firmware</h2>
  /// The Pentek 7142 and the associated SD3C firmware support four independent
  /// and identical channels. This class manages a single channel.
  ///
  /// In the firmware, each channel contains a downconvertor implemented with csac filters.
  /// The csac filters have a gaussian and a kaiser filter in series. The filter
  /// coefficients are configurable.
  ///
  /// In the firmware, he downconvertor is followed by user selectable processing
  /// modes. Each option configures the firmware timing differently, and
  /// delivers the data in a different format.
  ///
  /// A beam of data is defined as the I and Q values for one set of range gates.
  /// Baseband I and Q data may be either 16 bits or 32 bits, depending on the operating mode.
  /// Baseband data delivered to the host application via the read() system call.
  ///
  /// The modes are:
  /// <ul>
  /// <li>free running (fr): There is no gating of the received data. The baseband signal
  /// is output as a free running stream of 16 bit I and Q values.</li>
  /// <li> pulse tagger (pt): The downconversion is gated at the prt rate, and the specified
  /// number of gates is processed on each cycle. A synchronization word and a pulse count
  /// are prepended to the beam of baseband data.</li>
  /// <li> coherent integration (ci).: The downconversion is gated at the prt rate,
  /// and the specified
  /// number of gates is processed on each cycle. The prt is divided into even and odd
  /// pulses. I and values for even and odd pulses are accumulated
  /// separately for all gates. When the
  /// accumulation is finished, an  eight byte tag is prepended, and the I and Q
  /// summs for even and odd beams is output. </li>
  /// </ul>
  ///
  /// <h2>p7142sd3cdn usage</h2>
  /// A large part of this class is involved with configuring the SD3C firmware on the
  /// pentek card. Filter coefficients, timing parameters, and the data processing
  /// section must be configured properly.
  ///
  /// The other activity is the actual collection of the baseband data stream.
  /// Consumer applications call p7142sd3cdn to deliver I and Q data blocks, on a beam by beam
  /// basis. p7142sd3cdn handles detection and recovery from dropped data and synchronization
  /// errors. The consumer simply calls nextBeam() for the next beam of I and Q data.
  /// p7142sd3cdn will block until the request can be satisfied.
  ///
  /// Synchronization is performed when operating in the pulse tagging and
  /// coherent integration modes. Synchronization is required
  /// because it is possible for data to be dropped somewhere in the path from
  /// the SD3C to the delivery via the read() call. This synchronization
  /// can result in a certain amount of shifting and filling of
  /// buffers. Since any significant synchronization loss rate will not be acceptable in
  /// the system, we rely on the assumption that the number of re-syncs to be small, and these small
  /// scale buffer manipulations should not incur a significant overall processing cost.
  /// If re-synchronization occurs frequently, there is a hardware or design issue
  /// withthe system which must be corrected.
  ///
  /// <h2>Simulation</h2>
  /// For development and testing without the transceiver hardware, the p7142sd3cdn can
  /// be configured to operate in simulation mode. In this case, the p7142dn class
  /// is configured for simulation as well, and its read() function will synthesize
  /// simulated data. p7142sd3cdn will add sync words and tags as appropriate, depending on the
  /// operating mode. Synchronization errors are randomly inserted when operating in
  /// simulation mode.
  ///
  /// @todo It appears that the endianess of the pulse tagger sync word may
  /// not be accunted for here. If the sync code is ever changed to a non-symmetrical
  /// value, the simulation and synchronization code may break.
  ///
  /// <h2>Coherent Integrator</h2>
  /// The coherent integrator firmware in SD3C creates a data stream which is
  /// a little more complicated that the free run and pulse tagger modes. Becasue
  /// the coherent integrator tags cover 16 bytes, four of which contain fixed fields,
  /// this data stream does not contain separate sync words like the pulse tagger produces.
  /// The coherent integrator data format is documented in the SD3C VHDL as follows:
  /// @code
  /// --! <TAG_I_EVEN><TAG_Q_EVEN><TAG_I_ODD><TAG_Q_ODD><IQpairs,even pulse><IQpairs,odd pulse>
  /// --!
  /// --! The TAG is broken down as:
  /// --! bits 31:28  Format number   0-15(4 bits)
  /// --! bits 27:26  Channel number  0-3 (2 bits)
  /// --! bits    25  0=even, 1=odd   0-1 (1 bit)
  /// --! bit     24  0=I, 1=Q        0-1 (1 bit)
  /// --! bits 23:00  Sequence number     (24 bits)
  /// --!
  /// --!
  /// --! The four tags at the beginning should all have the same sequence number,
  /// --! verifying that the individual accumulators are working in sequence.
  /// --! The sequence number increments by one for each output sum from
  /// --! an accumulator.
  /// @endcode
  class p7142sd3cdn: public p7142dn {
  public:
    /// The type of downconverter that is instantiated in the pentek firmware.
    enum DDCDECIMATETYPE {
      DDC8DECIMATE, DDC4DECIMATE
    };

    /// Return the name of the given DDCDECIMATETYPE
    static std::string ddcTypeName(DDCDECIMATETYPE type) {
      switch (type) {
      case DDC8DECIMATE:
        return std::string("DDC8DECIMATE");
      case DDC4DECIMATE:
        return std::string("DDC4DECIMATE");
      default:
        return std::string("Unknown");
      }
    }

    /// The SD3C synchronization word value.
    static const uint32_t SD3C_SYNCWORD = 0xAAAAAAAA;
    /// The maximum pulse number in a pulse tagger tag
    static const int32_t MAX_PT_PULSE_NUM = 0x3FFFFFFF;
    /// The maximum pulse number in a coherent integrator tag
    static const int32_t MAX_CI_PULSE_NUM = 0xFFFFFF;

    /// Utility structure for timer initialization values
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
    /// @param simPauseMS The number of milliseconds to wait between beams
    /// simulated data when calling read()
    /// @param simWaveLength The wavelength of the simulated data, in sample counts
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
        double simPauseMS = 0.1,
        int simWaveLength = 5000,
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

    /// @return The data bandwidth from the pentek in bytes per second
    int dataRate();

    /// @return The ADC clock frequency in Hz.
    double adcFrequency() const {
      return _adc_clock;
    }
    /// Get one beam of data.
    /// @param pulsenum The pulse number is returned here. For raw data,
    /// the pulse number will be 0.
    /// @return A pointer to one beam of data.
    char* getBeam(unsigned int& pulsenum);
    /// @return The cumulative number of dropped pulses
    unsigned long droppedPulses();
    /// @return the number of synchronization errors detected.
    unsigned long syncErrors();

  protected:
    /// Configure the p7142sd3cdn. Used during construction.
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
    /// number of gates
    int _gates;
    /// number of coherent integrator sums
    int _nsum;
    //// The number of beams to preallocate in the buffer.
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

    // The data handling infrastructure.

    /// @returns The length in bytes of IQ data in each beam that the
    /// is returned by getBeam(). The user of p7142sd3cdn should
    /// verify that this matches their expectations. The beam length
    /// is computed during construction, and will not change thereafter.
    int beamLength();
    /// Initialize the buffer management. _beamSize will be computed.
    /// _buf will be allocated.
    void initBuffer();
    /// Return the next synchronized beam of pulse tagger data.
    /// The pulse number in the beam is checked for dropped beams.
    /// Data associated with synchronization errors will be skipped.
    /// The caller can access beamLength() bytes.
    /// @param pulsenum The pulse number is returned here.
    /// @returns Pointer to the start of the beam.
    char* ptBeamDecoded(unsigned int& pulseNum);
    /// Return the next synchronized beam of coherent integrator data.
    /// The pulse number in the beam is checked for dropped beams.
    /// Data associated with synchronization errors will be skipped.
    /// The caller can access beamLength() bytes.
    /// @param pulsenum The pulse number is returned here.
    /// @returns Pointer to the start of the beam.
    char* ciBeamDecoded(unsigned int& pulseNum);
    /// Return the next beam of free run data. This
    /// is a misnomer, since there aren't really beams in free run mode.
    /// Think of them as blocks. The caller can access beamLength() bytes.
    char* frBeam();
    /// Return the next synchronized beam of pulse tagger data.
    /// Data associated with synchronization errors will be skipped.
    /// The caller can access beamLength() bytes.
    /// @param pulseTag The pulse tag is returned here
    /// @returns Pointer to the start of the beam.
    char* ptBeam(char* pulseTag);
    /// Return the next synchronized beam of pulse tagger data.
    /// Data associated with synchronization errors will be skipped.
    /// The caller can access beamLength() bytes.
    /// @param pulseNum The pulse number is returned here
    /// @returns Pointer to the start of the beam.
    char* ciBeam(unsigned int& pulseNum);
    /// Decode the pulse coded data that come from the coherent integrator.
    /// Data are decoded from _buf into _ciBuf.
    void ciDecode();
    /// Check that a coherent integrator tag is
    /// valid.
    /// @param p Pointer to the tag.
    /// @param pulseNum The pulse number is returned here,
    /// if the tag is valid.
    /// @returns True if valid, false otherwise.
    bool ciCheckTag(char* p, unsigned int& pulseNum);
    /// Create a coherent integrator tag. Used for simulation.
    /// @param format The format identifier. Must match the format number produced by the firmware.
    /// @param chan The channel number (0-3).
    /// @param odd True for odd beam, false for an even
    /// @param Q   True for a Q beam, false for I data.
    /// @param seq The sequence number, from 0 to 0xffffff;
    uint32_t ciMakeTag(int format, int chan, bool odd, bool Q, uint32_t seq);
    /// Decode a coherent integrator tag.
    /// @param tag The tag to be decoded.
    /// @param format Returns the format identifier.
    /// @param chan Returns the channel number.
    /// @param odd Returns true for odd beam, false for an even
    /// @param Q   Returns true for a Q beam, false for I data.
    /// @param seq Returns the sequence number;
    void ciDecodeTag(uint32_t tag, int& format, int& chan, bool& odd, bool& Q, uint32_t& seq);
    /// Fill _simfifo with simulated data. Make
    /// sure that there are at least the specified number of bytes.
    /// Sync words and tags are added as appropriate. For
    /// PT and CI modes, occasional data drops are simulated.
    /// @param n The minimum number of bytes that _simFifo must
    /// contain on return from the routine.
    void makeSimData(int n);
    /// Perform the simulation wait. Called once per simulated beam.
    /// It will pause at a rate that is specified as the per beam
    /// wait time in simPauseMS. However, it will save up the pauses
    /// and do a 100x pause every 100 calls, so that the effective
    /// rate is close to the desired rate. Otherwise, the usleep() overhead
    /// kills us.
    void simWait();
    /// Decode the pulse tagger channel/pulse number word.
    /// @param buf A pointer to the channel/pulse number word.
    /// @param chan Return argument for the unpacked channel number.
    /// @param pulseNum Return argument for the unpacked pulse number.
    static void unpackPtChannelAndPulse(
        const char* buf,
        unsigned int & chan,
        unsigned int & pulseNum);
    /// Print the size and the leading data in _simFifo.
    /// @param label A label.
    /// @param n The number of items to print
    void dumpSimFifo(std::string label, int n);
    /// The three operating modes: free run, pulse tag and coherent integration
    enum {FR, PT, CI} _mode;
    /// The length of one beam of data, to be delivered
    /// on each getBeam() call.
    int _beamLength;
    /// The buffer that collects IQ data.
    char* _buf;
    /// A buffer that coherently integrated data will be decoded into.
    char* _ciBuf;
    /// The pulse number if we're simulating data
    int _simPulseNum;
    /// The number of ms to pause between each beam in simulation mode
    double _simPauseMS;
    /// The sim wait counter
    unsigned int _simWaitCounter;
    /// The last pulse sequence number that we received. Used to keep
    /// track of dropped pulses.
    int _lastPulse;
    /// An estimate of dropped pulses. It may be in error
    /// if the pulse tag rolls over by more than the 14 bit
    /// total that it can hold. This test is only made if the
    /// channel number passes the validity test.
    unsigned long _droppedPulses;
    /// The number of times that an incorrect channel number was received,
    /// which indicates a synchronization error. If there is a sync error,
    /// then the sequence number check is not performed.
    unsigned long _syncErrors;
    /// A flag used by the synchronization code to detect when the very first
    /// raw data are delivered from the card (or simulator).
    bool _firstRawBeam;
    /// Set false at startup, true after the first beam has been received.
    bool _firstBeam;
    /// A fifo that simulated data is built up by makeSimData() while in
    /// simulation mode. The read() function then draws out of this fifo.
    /// This is done so that makeSimData() can fill the buffer with
    /// multibyte objects such as shorts and ints, but the read function
    /// can draw out of it a byte at a time if necessary. This is required for
    /// re-synchroniation.
    std::deque<char> _simFifo;

  private:
    /// Open the control device.
    void openControlDevice();

  };

}

#endif /* P7172SD3C_H_ */
