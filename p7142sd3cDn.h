/*
 * p7142sd3cDn.h
 *
 *  Created on: Oct 5, 2010
 *      Author: burghart
 */

#ifndef P7142SD3CDN_H_
#define P7142SD3CDN_H_

#include "p7142Dn.h"
#include "FilterSpec.h"

#include <boost/date_time/posix_time/posix_time.hpp>

namespace Pentek {

class p7142sd3c;

/// @brief A p7142Dn downconverter class for P7142 cards running SD3C firmware.
///
/// In the firmware, each channel contains a downconverter implemented with csac
/// filters. The csac filters have a gaussian and a kaiser filter in series. The
/// filter coefficients are configurable.
///
/// In the firmware, the downconverter is followed by user selectable processing
/// modes. Each option configures the firmware timing differently, and
/// delivers the data in a different format.
///
/// A beam of data is defined as the I and Q values for one set of range gates.
/// Baseband I and Q data may be either 16 bits or 32 bits, depending on the 
/// operating mode. Baseband data delivered to the host application via the 
/// read() method.
///
/// The modes are:
/// <ul>
/// <li>free running (fr): There is no gating of the received data. The baseband
/// signal is output as a free running stream of 16 bit I and Q values.</li>
/// <li> pulse tagger (pt): The downconversion is gated at the prt rate, and the
/// specified number of gates is processed on each cycle. A synchronization word
/// and a pulse count are prepended to the beam of baseband data.</li>
/// <li> coherent integration (ci).: The downconversion is gated at the prt 
/// rate, and the specified number of gates is processed on each cycle. The prt
/// is divided into even and odd pulses. I and values for even and odd pulses 
/// are accumulated separately for all gates. When the accumulation is finished,
/// an  eight byte tag is prepended, and the I and Q summs for even and odd 
/// beams is output. </li>
/// </ul>
///
/// <h2>p7142sd3cDn usage</h2>
/// A large part of this class is involved with configuring the SD3C firmware on
/// the Pentek card. Filter coefficients, timing parameters, and the data 
/// processing section must be configured properly.
///
/// The other activity is the actual collection of the baseband data stream.
/// Consumer applications call p7142sd3cDn to deliver I and Q data blocks, on a 
/// beam by beam basis. The p7142sd3cDn class handles detection and recovery 
/// from dropped data and synchronization errors. The consumer simply calls 
/// nextBeam() for the next beam of I and Q data, and the p7142sd3cDn will block
/// until the request can be satisfied.
///
/// Synchronization is performed when operating in the pulse tagging and
/// coherent integration modes. Synchronization is required
/// because it is possible for data to be dropped somewhere in the path from
/// the SD3C to the delivery via the read() call. This synchronization
/// can result in a certain amount of shifting and filling of
/// buffers. Since any significant synchronization loss rate will not be 
/// acceptable in the system, we rely on the assumption that the number of 
/// re-syncs to be small, and these small scale buffer manipulations should not
/// incur a significant overall processing cost. If re-synchronization occurs 
/// frequently, there is a hardware or design issue with the system which must 
/// be corrected.
///
/// @todo It appears that the endianness of the pulse tagger sync word may
/// not be accounted for here. If the sync code is ever changed to a 
/// non-symmetrical value, the simulation and synchronization code may break.
///
/// <h2>Coherent Integrator</h2>
/// The coherent integrator firmware in SD3C creates a data stream which is
/// a little more complicated that the free run and pulse tagger modes. Because
/// the coherent integrator tags cover 16 bytes, four of which contain fixed 
/// fields, this data stream does not contain separate sync words like the pulse
/// tagger produces. The coherent integrator data format is documented in the 
/// SD3C VHDL as follows:
/// @code
/// --! <TAG_I_EVEN><TAG_Q_EVEN><TAG_I_ODD><TAG_Q_ODD><IQpairs,even pulse><IQpairs,odd pulse>
/// --!
/// --! The TAG is broken down as:
/// --! bits 31:28  Format number   0-15(4 bits)
/// --! bits 27:26  Channel number  0-3 (2 bits)
/// --! bit     25  0=even, 1=odd   0-1 (1 bit)
/// --! bit     24  0=I, 1=Q        0-1 (1 bit)
/// --! bits 23:00  Sequence number     (24 bits)
/// --!
/// --!
/// --! The four tags at the beginning should all have the same sequence number,
/// --! verifying that the individual accumulators are working in sequence.
/// --! The sequence number increments by one for each output sum from
/// --! an accumulator.
/// @endcode
class p7142sd3cDn : public p7142Dn {
public:
    /// Constructor.
    /// All times are expressed in seconds.
    /// @param p7142sd3cPtr Pointer to the p7142sd3c which owns this 
    ///     downconverter.
    /// @param chanId The channel identifier (0-3)
    /// @param burstSampling Set true if burst sampling should be used for this 
    ///     channel. Burst sampling implies that gates will be as short as the 
    ///     card's sampling clock will allow. The rx_pulsewidth and the sampling
    ///     clock frequency will determine the number of gates sampled.
    /// @param tsLength The number of pulses in one time series. Used to set 
    ///     interrupt buffer size, so that we have reasonable responsiveness in 
    ///     the data stream.
    /// @param rx_delay the delay to the first rx gate in seconds
    /// @param rx_pulsewidth the total length of time to sample data
    /// @param gaussianFile Name of the file containing the Gaussian filter 
    ///     parameters
    /// @param kaiserFile Name of the file containing the Kaiser filter 
    ///     parameters
    /// @param simPauseMS The number of milliseconds to wait between beams
    ///     simulated data when calling read()
    /// @param simWaveLength The wavelength of the simulated data, in sample counts
    /// @param internalClock Set true if the internal clock should be
    ///     used instead of an external clock source.
    p7142sd3cDn(
        p7142sd3c * p7142sd3cPtr, 
        int chanId, 
        bool burstSampling,
        int tsLength,
        double rx_delay, 
        double rx_pulsewidth,
        std::string gaussianFile, 
        std::string kaiserFile,
        double simPauseMS = 0.1,
        int simWaveLength = 5000,
        bool internalClock = false);
    
    /// Destructor
    virtual ~p7142sd3cDn();

    /// Read bytes. If in simulated mode, a sine wave with wavelength
    /// of _simWaveLength gates will be synthesized. It will have some 
    /// random noise applied as well.
    /// @param buf read bytes into this buffer
    /// @param bufsize The number of bytes to read.
    /// @return The actual number of bytes read
    virtual int read(char* buf, int bufsize);
    
    /// @return The receiver pulsewidth, in s
    double rcvrPulseWidth() const;
    
    /// @return The receiver delay to first gate, in s
    double rcvrFirstGateDelay() const;
    
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
    boost::posix_time::ptime timeOfPulse(int64_t nPulsesSinceStart) const;
    
    /// @return The closest pulse number to a given time.
    int64_t pulseAtTime(boost::posix_time::ptime time) const;
    
    /// @returns The length in bytes of IQ data in each beam that the
    /// is returned by getBeam(). The user of p7142sd3cDn should
    /// verify that this matches their expectations. The beam length
    /// is computed during construction, and will not change thereafter.
    int beamLength();
    
    /// Get one or two beams of data. For free run and
    /// pulse tagger mode, one beam is returned. For the coherent integrator
    /// mode, an even and an odd beam are returned.
    /// @param nPulsesSinceStart[out] the number of pulses since the
    ///   xmitter was started up - allows computation of the time
    /// @return A pointer to one beam of data.
    char* getBeam(int64_t& nPulsesSinceStart);
    
    /// Return our gate count. For burst sampling channels, this may be
    /// different from the gate count set for our p7142sd3c object.
    /// @return the gate count for this downconverter.
    unsigned int gates() const { return _gates; }
    
    /// @return The cumulative number of dropped pulses
    unsigned long droppedPulses();
    /// @return the number of synchronization errors detected.
    unsigned long syncErrors();
    /// Return the name of the DDC type being used.
    std::string ddcTypeName() const;
    /// Return the estimated data interrupt period, which is a good estimate
    /// of the maximum data latency time.
    double dataInterruptPeriod() const { return _dataInterruptPeriod; }

protected:

    /// Configure the p7142sd3cDn. Used during construction.
    /// @return True if the configuration was successful
    bool config();

    /// Configure the down converter fifos
    void fifoConfig();

    /// Configure the filters and the decimation value.
    int filterSetup();

    /// Program the coefficients for the Gaussian and Kaiser filters.
    bool loadFilters(FilterSpec& gaussian, FilterSpec& kaiser);

    /// Set the interrupt buffer size for the pentek. Useful
    /// for trying to control the interrupt rate, based on how
    /// fast the FPGA is running.
    /// @todo We don't currently use this, because there were
    /// indications that it was causing the driver to drop
    /// blocks of data.
    void setInterruptBufSize();

    // The data handling infrastructure.

    /// Initialize the buffer management. _beamSize will be computed.
    /// _buf will be allocated.
    void initBuffer();
    /// Return the next synchronized beam of pulse tagger data.
    /// The pulse number in the beam is checked for dropped beams.
    /// Data associated with synchronization errors will be skipped.
    /// The caller can access beamLength() bytes.
    /// @param nPulsesSinceStart: the number of pulses since the
    ///   xmitter was started up - allows computation of the time
    /// @returns Pointer to the start of the beam.
    char* ptBeamDecoded(int64_t& nPulsesSinceStart);
    /// Return the next synchronized beam of coherent integrator data.
    /// The pulse number in the beam is checked for dropped beams.
    /// Data associated with synchronization errors will be skipped.
    /// The caller can access beamLength() bytes.
    /// @param nPulsesSinceStart: the number of pulses since the
    ///   xmitter was started up - allows computation of the time
    /// @returns Pointer to the start of the beam.
  char* ciBeamDecoded(int64_t& nPulsesSinceStart);
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
    /// Check that a coherent integrator tag is
    /// valid.
    /// @param p Pointer to the tag.
    /// @param pulseNum The pulse number is returned here,
    /// if the tag is valid.
    /// @returns True if valid, false otherwise.
    static bool ciCheckTag(char* p, unsigned int& pulseNum);
    /// Create a coherent integrator tag. Used for simulation.
    /// @param format The format identifier. Must match the format number produced by the firmware.
    /// @param chan The channel number (0-3).
    /// @param odd True for odd beam, false for an even
    /// @param Q   True for a Q beam, false for I data.
    /// @param seq The sequence number, from 0 to 0xffffff;
    static uint32_t ciMakeTag(int format, int chan, bool odd, bool Q, uint32_t seq);
    /// Decode a coherent integrator tag.
    /// @param tag The tag to be decoded.
    /// @param format Returns the format identifier.
    /// @param chan Returns the channel number.
    /// @param odd Returns true for odd beam, false for an even
    /// @param Q   Returns true for a Q beam, false for I data.
    /// @param seq Returns the sequence number;
    static void ciDecodeTag(uint32_t tag, int& format, int& chan, bool& odd, 
            bool& Q, uint32_t& seq);
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
    
    /// The SD3C synchronization word value.
    static const uint32_t SD3C_SYNCWORD = 0xAAAAAAAA;
    
    /// The maximum pulse number in a pulse tagger tag
    static const int32_t MAX_PT_PULSE_NUM = 0x3FFFFFFF;
    
    /// The maximum pulse number in a coherent integrator tag
    static const int32_t MAX_CI_PULSE_NUM = 0xFFFFFF;
    
    /// The p7142sd3c which owns us
    p7142sd3c & _sd3c;
    /// Is this a burst sampling channel?
    bool _isBurst;
    /// number of gates
    int _gates;
    /// number of coherent integrator sums
    int _nsum;
    //// The number of beams to preallocate in the buffer.
    int _tsLength;
    /// The path to the file containing the gaussian filter definitions.
    std::string _gaussianFile;
    /// The path to the file containing the kaiser filter coefficients.
    std::string _kaiserFile;
    /// The length of one beam of data, to be delivered
    /// on each getBeam() call.
    int _beamLength;
    /// The buffer that collects IQ data.
    char* _buf;
    /// The pulse number if we're simulating data
    int _simPulseNum;
    /// The number of ms to pause between each beam in simulation mode
    double _simPauseMS;
    /// The sim wait counter
    unsigned int _simWaitCounter;
    /// The last pulse sequence number that we received. Used to keep
    /// track of dropped pulses.
    int _lastPulse;
    /// The number of pulses since the xmitter was started.
    /// In conjunction with the PRF and start time, this allows the
    /// pulse time to be computed. See timeOfPulse().
    int64_t _nPulsesSinceStart;
    /// An estimate of dropped pulses. It may be in error
    /// if the pulse tag rolls over by more than the
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
    /// Estimated period between data interrupts, in seconds
    /// This provides a reasonable guess at the maximum data latency time
    /// for the channel.
    double _dataInterruptPeriod;
    /// A fifo that simulated data is built up by makeSimData() while in
    /// simulation mode. The read() function then draws out of this fifo.
    /// This is done so that makeSimData() can fill the buffer with
    /// multibyte objects such as shorts and ints, but the read function
    /// can draw out of it a byte at a time if necessary. This is required for
    /// re-synchronization.
    std::deque<char> _simFifo;
};

} // end namespace Pentek
#endif /* P7142SD3CDN_H_ */
