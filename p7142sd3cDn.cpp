/*
 * p7142sd3cDn.cpp
 *
 *  Created on: Oct 5, 2010
 *      Author: burghart
 */
#include "p7142sd3c.h"
#include "p7142sd3cDn.h"
#include "BuiltinGaussian.h"
#include "BuiltinKaiser.h"
#include "FilterSpec.h"
#include <sys/ioctl.h>
#include <cerrno>
#include <cmath>

using namespace boost::posix_time;

namespace Pentek {

////////////////////////////////////////////////////////////////////////////////
p7142sd3cDn::p7142sd3cDn(p7142sd3c * p7142sd3cPtr, int chanId, 
        bool burstSampling, int tsLength, double rx_delay, double rx_pulsewidth,
        std::string gaussianFile, std::string kaiserFile, double simPauseMS, 
        int simWaveLength, bool internalClock) :
        p7142Dn(p7142sd3cPtr, 
                chanId, 
                1, 
                simWaveLength,
                p7142sd3cPtr->nsum() > 1,
                internalClock),
        _sd3c(*p7142sd3cPtr),
        _isBurst(burstSampling),
        _tsLength(tsLength),
        _gaussianFile(gaussianFile), 
        _kaiserFile(kaiserFile),
        _simPulseNum(0),
        _simPauseMS(simPauseMS),
        _simWaitCounter(0),
        _lastPulse(0),
        _nPulsesSinceStart(0),
        _droppedPulses(0),
        _syncErrors(0),
        _firstRawBeam(true),
        _firstBeam(true),
        _dataInterruptPeriod(0.0)
{
    boost::recursive_mutex::scoped_lock guard(_mutex);

    // Get gate count and coherent integration sum count from our card
    _gates = _sd3c.gates();
    _nsum = _sd3c.nsum();
    
    // Convert our rx delay and width to counts.
    int rxDelayCounts = _sd3c.timeToCounts(rx_delay);
    int rxPulsewidthCounts = _sd3c.timeToCounts(rx_pulsewidth);
    if (rxPulsewidthCounts == 0) {
        std::cerr << "Rx pulsewidth of " << rx_pulsewidth << " seconds " <<
                "for channel " << _chanId << 
                " is zero counts @ ADC clock freq of " << _sd3c.adcFrequency() <<
                " Hz!" << std::endl;
        abort();
    }

    // Set the rx gating timer. 
    // Note that Channels 0 and 1 share RX_01_TIMER, and channels 2 and 3 
    // share RX_23_TIMER.
    // For a burst sampling channel, take as many gates as the clock allows 
    // over the given pulse width, and set _gates to the correct value here.
    p7142sd3c::TimerIndex rxTimerNdx = (_chanId <= 1) ? 
            p7142sd3c::RX_01_TIMER : p7142sd3c::RX_23_TIMER;
    if (_isBurst) {
        _gates = rxPulsewidthCounts;
        _sd3c._setTimer(rxTimerNdx, rxDelayCounts, rxPulsewidthCounts);
    } else {
        _sd3c._setTimer(rxTimerNdx, rxDelayCounts, rxPulsewidthCounts * _gates);
    }
    
    // Estimate the period between data-available interrupts based on the 
    // configured interrupt buffer length. This is a good first-order estimate
    // of maximum data latency time for the channel.
    // TODO: Configure the channel intbufsize for roughly 10 Hz interrupts
    // (and bufsize to ~2*intbufsize)
    BUFFER_CFG bufConfig;
    if (isSimulating()) {
        bufConfig.intbufsize = 32768;
        bufConfig.bufsize = 524288;
    } else {
        ioctl(fd(), BUFGET, &bufConfig);
    }
    
    int interruptBytes = 4 * bufConfig.intbufsize;  // intbufsize is in 4-byte words
    double chanDataRate = (4 * _gates) / _sd3c.prt();   // @TODO this only works for single PRT
    _dataInterruptPeriod = interruptBytes / chanDataRate;

    // Warn if data latency is greater than 1 second, and bail completely if
    // it's greater than 5 seconds.
    if (_dataInterruptPeriod > 1.0) {
        std::cerr << "interruptBytes " << interruptBytes << std::endl;
        std::cerr << "chanDataRate " << chanDataRate << std::endl;
        std::cerr << "Warning: Estimated max data latency for channel " << 
        _chanId << " is " << _dataInterruptPeriod << " s!" << std::endl;
    }
    if (_dataInterruptPeriod > 5.0) {
        abort();
    }
    
    // initialize the buffering scheme.
    initBuffer();

    if (isSimulating())
        return;

    /// Set the bypass divider (decimation) for our receiver channel
    int bypassOk = _isBurst ? 
            setBypassDivider(2) : setBypassDivider(2 * rxPulsewidthCounts);
    if (!bypassOk) {
        std::cerr << "Failed to set decimation for channel " << _chanId << 
                std::endl;
        abort();
    }
    std::cout << "bypass decim: " << bypassDivider() << std::endl;
    
    // flush the fifos. Note that a flush must not be issued
    // after the timers have been configured, as this will zero
    // the timer parameters.
    flush();

    // configure DDC in FPGA
    if (!config()) {
        std::cout << "error initializing filters\n";
    }

}

////////////////////////////////////////////////////////////////////////////////
p7142sd3cDn::~p7142sd3cDn() {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    delete [] _buf;
}

////////////////////////////////////////////////////////////////////////////////
std::string p7142sd3cDn::ddcTypeName() const {
    return p7142sd3c::ddcTypeName(_sd3c.ddcType());
}

////////////////////////////////////////////////////////////////////////////////
double p7142sd3cDn::rcvrPulseWidth() const {
    boost::recursive_mutex::scoped_lock guard(_mutex);
    // Note that Channels 0 and 1 share RX_01_TIMER, and channels 2 and 3 
    // share RX_23_TIMER.
    p7142sd3c::TimerIndex rxTimerNdx = (_chanId <= 1) ? 
            p7142sd3c::RX_01_TIMER : p7142sd3c::RX_23_TIMER;
    return(_sd3c.countsToTime(_sd3c._timerWidth(rxTimerNdx)));
}

////////////////////////////////////////////////////////////////////////////////
double p7142sd3cDn::rcvrFirstGateDelay() const {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    int txDelayCounts = _sd3c._timerDelay(p7142sd3c::TX_PULSE_TIMER);
    // Note that Channels 0 and 1 share RX_01_TIMER, and channels 2 and 3 
    // share RX_23_TIMER.
    p7142sd3c::TimerIndex rxTimerNdx = (_chanId <= 1) ? 
            p7142sd3c::RX_01_TIMER : p7142sd3c::RX_23_TIMER;
    int rxDelayCounts = _sd3c._timerDelay(rxTimerNdx);

    return(_sd3c.countsToTime(rxDelayCounts - txDelayCounts));
}
////////////////////////////////////////////////////////////////////////////////
bool p7142sd3cDn::config() {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    // configure the fifo
    fifoConfig();

    // Stop the filters from running
    _sd3c.stopFilters();

    // Is coherent integration enabled?
    std::cout << "coherent integration is " <<
          (_nsum > 1 ? "enabled" : "disabled") << std::endl;

    // set up the filters. Will do nothing if either of
    // the filter file paths is empty or if this is a burst channel
    bool filterError = filterSetup();
    if (filterError) {
        return false;
    }

    return true;
}

//////////////////////////////////////////////////////////////////////
bool p7142sd3cDn::loadFilters(FilterSpec& gaussian, FilterSpec& kaiser) {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (isSimulating())
        return true;
    
    int ddcSelect = _chanId << 14;

    // program kaiser coefficients

    bool kaiserLoaded = true;
    
    for (unsigned int i = 0; i < kaiser.size(); i++) {

        // Set up to write this coefficient
        int ramAddr = 0;
        int ramSelect = 0;
        switch (_sd3c.ddcType()) {
        case p7142sd3c::DDC10DECIMATE:
            ramAddr = i / 10;
            ramSelect = (i % 10) << 4;
            break;
        case p7142sd3c::DDC8DECIMATE:
            ramAddr = i / 8;
            ramSelect = (i % 8) << 4;
            break;
        case p7142sd3c::DDC4DECIMATE:
            ramAddr = i / 4;
            ramSelect = (i % 4) << 4;
            break;
        case p7142sd3c::BURST:   // Burst mode uses no filters
            break;    
        }
        
        _sd3c._controlIoctl(FIOREGSET, KAISER_ADDR, 
                ddcSelect | DDC_STOP | ramSelect | ramAddr);

        // Try up to a few times to program this filter coefficient and
        // read it back successfully.
        bool coeffLoaded = false;
        for (int attempt = 0; attempt < 5; attempt++) {
            // write the value
            // LS word first
            _sd3c._controlIoctl(FIOREGSET, KAISER_DATA_LSW, kaiser[i] & 0xFFFF);
    
            // then the MS word -- since coefficients are 18 bits and FPGA 
            // registers are 16 bits!
            _sd3c._controlIoctl(FIOREGSET, KAISER_DATA_MSW, 
                    (kaiser[i] >> 16) & 0x3);
    
            // latch coefficient
            _sd3c._controlIoctl(FIOREGSET, KAISER_WR, 0x1);
    
            // disable writing (kaiser readback only succeeds if we do this)
            _sd3c._controlIoctl(FIOREGSET, KAISER_WR, 0x0);
    
            // read back the programmed value; we need to do this in two words 
            // as above.
            unsigned int readBack;
            readBack = _sd3c._controlIoctl(FIOREGGET, KAISER_READ_LSW) | 
                   (_sd3c._controlIoctl(FIOREGGET, KAISER_READ_MSW) << 16);

            if (readBack == kaiser[i]) {
                coeffLoaded = true;
                if (attempt != 0) {
                    std::cout << ":" << std::hex << readBack << std::dec <<
                            " -- OK" << std::endl;
                }
                break;
            } else {
                if (attempt == 0) {
                    std::cout << "kaiser[" << i << "] = " << std::hex <<
                            kaiser[i] << ", readbacks: " << readBack <<
                            std::dec;
                } else {
                    std::cout << ":" << std::hex << readBack << std::dec;
                }
            }
        }
        if (! coeffLoaded) {
            std::cout << " -- FAILED!" << std::endl;
        }
        
        kaiserLoaded |= coeffLoaded;
    }

    if (kaiserLoaded) {
        std::cout << kaiser.size()
                << " Kaiser filter coefficients successfully loaded" << std::endl;
    } else {
        std::cout << "Unable to load the Kaiser filter coefficients" << std::endl;
    }

    // program gaussian coefficients
    // Note that the DDC select is accomplished in the kaiser filter coefficient
    // address register, which was done during the previous kaiser filter load.

    bool gaussianLoaded = true;
    
    for (unsigned int i = 0; i < gaussian.size(); i++) {

        // Set up to write this coefficient
        int ramAddr = 0;
        int ramSelect = 0;
        switch (_sd3c.ddcType()) {
        case p7142sd3c::DDC10DECIMATE:
            ramAddr = i % 10;
            ramSelect = (i / 10) << 4;
            break;
        case p7142sd3c::DDC8DECIMATE:
            ramAddr = i % 8;
            ramSelect = (i / 8) << 4;
            break;    
        case p7142sd3c::DDC4DECIMATE:
            ramAddr = i % 12;
            ramSelect = (i / 12) << 4;
            break;
        case p7142sd3c::BURST:   // Burst mode uses no filters
            break;    
        }
        /// @todo early versions of the gaussian filter programming required
        /// the ds select bits to be set in the gaussian address register.
        /// We can take this out when we get a working bitstream with this
        /// fixed

        // Try up to a few times to program this filter coefficient and
        // read it back successfully.
        bool coeffLoaded = false;
        for (int attempt = 0; attempt < 5; attempt++) {
            // set the address
            _sd3c._controlIoctl(FIOREGSET, GAUSSIAN_ADDR, 
                    ddcSelect | ramSelect | ramAddr);
    
            // write the value
            // LS word first
            _sd3c._controlIoctl(FIOREGSET, GAUSSIAN_DATA_LSW, 
                    gaussian[i] & 0xFFFF);
    
            // then the MS word -- since coefficients are 18 bits and FPGA 
            // registers are 16 bits!
            _sd3c._controlIoctl(FIOREGSET, GAUSSIAN_DATA_MSW, 
                    (gaussian[i] >> 16) & 0x3);
    
            // latch coefficient
            _sd3c._controlIoctl(FIOREGSET, GAUSSIAN_WR, 0x1);
    
            // disable writing (gaussian readback only succeeds if we do this)
            _sd3c._controlIoctl(FIOREGSET, GAUSSIAN_WR, 0x0);
    
            // read back the programmed value; we need to do this in two words 
            // as above.
            unsigned int readBack;
            readBack = _sd3c._controlIoctl(FIOREGGET, GAUSSIAN_READ_LSW) |
                    (_sd3c._controlIoctl(FIOREGGET, GAUSSIAN_READ_MSW) << 16);
            if (readBack == gaussian[i]) {
                coeffLoaded = true;
                if (attempt != 0) {
                    std::cout << ":" << std::hex << readBack << std::dec <<
                            " -- OK" << std::endl;
                }
                break;
            } else {
                if (attempt == 0) {
                    std::cout << "gaussian[" << i << "] = " << std::hex <<
                            gaussian[i] << ", readbacks: " << readBack <<
                            std::dec;
                } else {
                    std::cout << ":" << std::hex << readBack << std::dec;
                }
            }
        }
        if (! coeffLoaded) {
            std::cout << " -- FAILED!" << std::endl;
        }
        
        gaussianLoaded |= coeffLoaded;
    }

    if (gaussianLoaded) {
        std::cout << gaussian.size()
                << " Gaussian filter coefficients successfully loaded" << std::endl;
    } else {
        std::cout << "Unable to load the Gaussian filter coefficients" << std::endl;
    }

    // return to decimal output
    std::cout << std::dec;

    return kaiserLoaded && gaussianLoaded;

}
////////////////////////////////////////////////////////////////////////

int p7142sd3cDn::filterSetup() {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    // No filters if this is a burst sampling channel
    if (_isBurst)
        return 0;

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
        double pulsewidthUs = 1.00;
        gaussianFilterName = "ddc8_1_0";

        // Choose the correct builtin Gaussian filter coefficient set.
        switch (_sd3c.ddcType()) {
        case p7142sd3c::DDC8DECIMATE: {
            switch ((int)(_sd3c.countsToTime(_sd3c._timerWidth(p7142sd3c::TX_PULSE_TIMER)) * 1.0e7)) {

            case 2:                             //pulse width = 0.256 microseconds
                pulsewidthUs = 0.256;
                gaussianFilterName = "ddc8_0_2";
                break;
            case 3:                             //pulse width = 0.384 microseconds
                pulsewidthUs = 0.384;
                gaussianFilterName = "ddc8_0_3";
                break;
            case 5:
                pulsewidthUs = 0.512;           //pulse width = 0.512 microseconds
                gaussianFilterName = "ddc8_0_5";
                break;
            case 6:                             //pulse width = 0.64 microseconds
                pulsewidthUs = 0.64;
                gaussianFilterName = "ddc8_0_6";
                break;
            case 7:                             //pulse width = 0.768 microseconds
                pulsewidthUs = 0.768;
                gaussianFilterName = "ddc8_0_7";
                break;
            case 8:                             //pulse width = 0.896 microseconds
                pulsewidthUs = 0.896;
                gaussianFilterName = "ddc8_0_8";
                break;
            case 10:                            //pulse width = 1.024 microseconds
                pulsewidthUs = 1.024;
                gaussianFilterName = "ddc8_1_0";
                break;
            default:
                std::cerr << "chip width specification of "
                          << _sd3c._timerWidth(p7142sd3c::TX_PULSE_TIMER)
                          << " is not recognized, filter will be configured for a "
                          << pulsewidthUs << " uS pulse\n";
                break;
            }
            break;
        }
        case p7142sd3c::DDC4DECIMATE: {    // pulse_width in 24 MHz counts
            pulsewidthUs = 1.0;
            gaussianFilterName = "ddc4_1_0";
            break;
        }
        case p7142sd3c::DDC10DECIMATE: {    // pulse_width in 50 MHz counts
            pulsewidthUs = 0.5;
//            gaussianFilterName = "ddc10_0_5";
            gaussianFilterName = "ddc10_0_5_flat";
            break;
        }
        default: {
            std::cerr << "DDC type " << ddcTypeName() << 
                " not handled in " << __FUNCTION__ << std::endl;
            abort();
        }
        }

        if (builtins.find(gaussianFilterName) == builtins.end()) {
            std::cerr << "No entry for " << gaussianFilterName << ", "
                    << pulsewidthUs
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
        switch (_sd3c.ddcType()) {
        case p7142sd3c::DDC8DECIMATE: {
            kaiserFilterName = "ddc8_5_0";
            break;
        }
        case p7142sd3c::DDC4DECIMATE: {
            kaiserFilterName = "ddc4_4_0";
            break;
        }
        case p7142sd3c::DDC10DECIMATE: {
            kaiserFilterName = "ddc10_5_0";
            break;
        }
        default: {
            std::cerr << "DDC type " << ddcTypeName() << 
                " not handled in " << __FUNCTION__ << std::endl;
            abort();
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
        if (! usingInternalClock()) {
            std::cerr << "Is the external clock source connected?" << std::endl;
            std::cerr << "Is the clock signal strength at least +3 dBm?" << 
                std::endl;
        }
        exit(1);
    }

    return 0;
}

//////////////////////////////////////////////////////////////////////
void p7142sd3cDn::setInterruptBufSize() {
    boost::recursive_mutex::scoped_lock guard(_mutex);

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

    double tsFreq = _sd3c._prf / _tsLength;

    // we want the interrupt buffer size to be a multiple of tsBlockSize,
    // but no more than 20 interrupts per second.
    int intBlocks = 1;

    if (tsFreq <= 20) {
        intBlocks = 1;
    } else {
        intBlocks = (int)(tsFreq / 20) + 1;
    }

    int bufferSize = tsBlockSize * intBlocks;

    std::cout << "prt is " << _sd3c._prtCounts << "  prt frequency is " << 
            _sd3c._prf << "  ts freq is " << tsFreq << 
            "  tsblocks per interrupt is " << intBlocks << std::endl;

    std::cout << "pentek interrupt buffer size is " << bufferSize << std::endl;

    // set the buffer size
    _sd3c.bufset(_dnFd, bufferSize, 2);

}

//////////////////////////////////////////////////////////////////////
void p7142sd3cDn::fifoConfig() {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (isSimulating())
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

    readBack = _sd3c._controlIoctl(FIOREGGET, ppOffset);

    // And configure ADC FIFO Control for this channel
    _sd3c._controlIoctl(FIOREGSET, ppOffset, readBack & 0x000034BF);

}

//////////////////////////////////////////////////////////////////////////////////
//
// ******    Buffer management and data handling in the following section    *****
//
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
int
p7142sd3cDn::read(char* buf, int n) {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    // Unless we're simulating, we just use the superclass read
    if (!isSimulating()) {
        int r =  p7142Dn::read(buf, n);
        assert(r == n);
        /**
        std::cout << "read " << r << " bytes" << std::endl;
        std::cout << std::hex;
	    for (unsigned int i = 0; i < (unsigned int)r; i++) {
	        std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)(unsigned char)buf[i] << " ";
	        if (!((i+1) % 40)) {
	            std::cout << std::endl;
	        }
	    }
	    std::cout << std::dec << std::endl;;
	    **/
        return r;
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
p7142sd3cDn::getBeam(int64_t& nPulsesSinceStart) {

    // perform the simulation wait if necessary
    if (isSimulating()) {
        simWait();
    }

    switch (_sd3c._operatingMode()) {
        case p7142sd3c::MODE_FREERUN:
            nPulsesSinceStart = 0;
            return frBeam();
        case p7142sd3c::MODE_PULSETAG:
            return ptBeamDecoded(nPulsesSinceStart);
        case p7142sd3c::MODE_CI:
            return ciBeamDecoded(nPulsesSinceStart);
        default:
            std::cerr << __PRETTY_FUNCTION__ << ": unhandled mode " << 
                _sd3c._operatingMode() << std::endl;
            abort();
    }
}

//////////////////////////////////////////////////////////////////////////////////
int
p7142sd3cDn::beamLength() {
    boost::recursive_mutex::scoped_lock guard(_mutex);
    return _beamLength;
}

//////////////////////////////////////////////////////////////////////////////////
char*
p7142sd3cDn::ptBeamDecoded(int64_t& nPulsesSinceStart) {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    // get the beam
    char pulseTag[4];
    char* buf = ptBeam(pulseTag);

    // unpack the channel number and pulse sequence number.
    // Unpack the 4-byte channel id/pulse number
    unsigned int chan, pulseNum;
    unpackPtChannelAndPulse(pulseTag, chan, pulseNum);
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
    if (delta < (-MAX_PT_PULSE_NUM / 2)) {
        // if the new pulse number is zero, assume that it
        // was a legitimate wrap. Unfortunately this won't catch
        // errors where the zero pulse is skipped, or a pulse comes in
        // that erroneously has zero for a pulse tag. Perhaps there
        // is a better algorithm for this.
        if (pulseNum == 0)
            std::cout << "Pulse number rollover" << std::endl;

        delta += MAX_PT_PULSE_NUM + 1;
    }

    if (delta == 0) {
        std::cerr << "Channel " << _chanId << ": got repeat of pulse " <<
                pulseNum << "!" << std::endl;
        abort();
    } else if (delta != 1) {
        // std::cerr << _lastPulse << "->" << pulseNum << ": ";
        if (delta < 0) {
            //std::cerr << "Channel " << _chanId << " went BACKWARD " <<
            //    -delta << " pulses" << std::endl;
        } else {
            // std::cerr << "Channel " << _chanId << " dropped " <<
            //    delta - 1 << " pulses" << std::endl;
        }
    }

    _nPulsesSinceStart += delta;
    nPulsesSinceStart = _nPulsesSinceStart;

    _droppedPulses += (delta - 1);
    _lastPulse = pulseNum;

    return buf;
}
//////////////////////////////////////////////////////////////////////////////////
char*
p7142sd3cDn::ptBeam(char* pulseTag) {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    int r;
    while(1) {
        if (_firstRawBeam) {
            // skip over the first 4 bytes, assuming that
            // they are a good sync word.
            r = read(_buf, 4);
            assert(r == 4);
            _firstRawBeam = false;
        }

        // read pulse number
        r = read(pulseTag, 4);
        assert(r == 4);

        // read one beam of IQ data into buf
        r = read(_buf, _beamLength);
        assert(r == (_beamLength));

        // read the next sync word
        uint32_t sync;
        r = read(reinterpret_cast<char*>(&sync), 4);
        assert(r == 4);

        // If we are indeed in sync, return the good pulse data now
        if (sync == SD3C_SYNCWORD)
            return _buf;
            
        // No sync? Hunt word-by-word until we find a sync word, then go 
        // back to the top
        _syncErrors++;
        while (sync != SD3C_SYNCWORD) {
            r = read(reinterpret_cast<char*>(&sync), 4);
            assert(r == 4);
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////
char*
p7142sd3cDn::ciBeamDecoded(int64_t& nPulsesSinceStart) {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    // get the beam
    unsigned int pulseNum;
    char* buf = ciBeam(pulseNum);

    // Initialize _lastPulse if this is the first pulse we've seen
    if (_firstBeam) {
        _lastPulse = pulseNum - 1;
        _firstBeam = false;
    }

    // How many pulses since the last one we saw?
    int delta = pulseNum - _lastPulse;
    if (delta < (-MAX_CI_PULSE_NUM / 2)) {
        // if the new pulse number is zero, assume that it
        // was a legitimate wrap. Unfortunately this won't catch
        // errors where the zero pulse is skipped, or a pulse comes in
        // that erroneously has zero for a pulse tag. Perhaps there
        // is a better algorithm for this.
        if (pulseNum == 0)
            std::cout << "Pulse number rollover" << std::endl;

        delta += MAX_CI_PULSE_NUM + 1;

    }

    if (delta == 0) {
        std::cerr << "Channel " << _chanId << ": got repeat of pulse " <<
                pulseNum << "!" << std::endl;
        abort();
    } else if (delta != 1) {
        //std::cerr << _lastPulse << "->" << pulseNum << ": ";
        if (delta < 0) {
            //std::cerr << "Channel " << _chanId << " went BACKWARD " <<
            //    -delta << " pulses" << std::endl;
        } else {
            //std::cerr << "Channel " << _chanId << " dropped " <<
            //    delta - 1 << " pulses" << std::endl;
        }
    }

    _nPulsesSinceStart += delta;
    nPulsesSinceStart = _nPulsesSinceStart;

    _droppedPulses += (delta - 1);
    _lastPulse = pulseNum;

    return buf;
}
//////////////////////////////////////////////////////////////////////////////////
char*
p7142sd3cDn::ciBeam(unsigned int& pulseNum) {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    int r;

    while(1) {
        if (_firstRawBeam) {
            // skip over the first 16 bytes, assuming that
            // they are a good tag word.
            r = read(_buf, 16);
            assert(r == 16);
            _firstRawBeam = false;
        }
        // read one beam into buf
        r = read(_buf, _beamLength);
        assert(r == _beamLength);

        // read the next tag word
        char tagbuf[16];
        r = read(tagbuf, 16);
        assert(r == 16);

        if (ciCheckTag(tagbuf, pulseNum)) {
            return _buf;
        }
        _syncErrors++;
        
        // scan 4 bytes at a time for a correct tag
        while(1) {
            memmove(tagbuf, tagbuf+4,12);
            r = read(tagbuf+12, 4);
            assert(r == 4);
            // check for synchronization
            if (ciCheckTag(tagbuf, pulseNum)) {
                break;
            }
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////
char*
p7142sd3cDn::frBeam() {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    int r = read(_buf, _beamLength);
    assert(r == _beamLength);
    return _buf;
}

//////////////////////////////////////////////////////////////////////////////////
bool
p7142sd3cDn::ciCheckTag(char* p, unsigned int& pulseNum) {

/// The tag order:
///  --! <TAG_I_EVEN><TAG_Q_EVEN><TAG_I_ODD><TAG_Q_ODD><IQpairs,even pulse><IQpairs,odd pulse>
///
/// The CI tag:
///  --! bits 31:28  Format number   0-15(4 bits)
///  --! bits 27:26  Channel number  0-3 (2 bits)
///  --! bits    25  0=even, 1=odd   0-1 (1 bit)
///  --! bit     24  0=I, 1=Q        0-1 (1 bit)
///  --! bits 23:00  Sequence number     (24 bits)

    int format[4];
    int chan[4];
    bool Odd[4];
    bool Q[4];
    uint32_t seq[4];
    for (int i = 0; i < 4; i++) {
        uint32_t* tag = (uint32_t*)p;
        ciDecodeTag(tag[i], format[i], chan[i], Odd[i], Q[i], seq[i]);
    }

    pulseNum = seq[0];

    // time to see if we received expected values
    bool retval = true;

    retval     = retval && (format[0] ==      1);

    for (int i = 1; i < 4; i++) {
        retval = retval && (format[i] ==      1);
        retval = retval && (seq[i]    == seq[0]);
        retval = retval && (chan[i]   == chan[0]);
    }
    retval = retval && !Odd[0] && !Odd[1] && Odd[2] && Odd[3];
    retval = retval &&   !Q[0] &&    Q[1] &&  !Q[2] &&   Q[3];

    return retval;;
}

//////////////////////////////////////////////////////////////////////////////////
uint32_t
p7142sd3cDn::ciMakeTag(int format, int chan, bool odd, bool Q, uint32_t seq) {
    /// The CI tag:
    ///  --! bits 31:28  Format number   0-15(4 bits)
    ///  --! bits 27:26  Channel number  0-3 (2 bits)
    ///  --! bits    25  0=even, 1=odd   0-1 (1 bit)
    ///  --! bit     24  0=I, 1=Q        0-1 (1 bit)
    ///  --! bits 23:00  Sequence number     (24 bits)

    int Odd = odd? 1:0;
    int IQ   =  Q? 1:0;
    uint32_t tag =
    		(( format << 4 | chan << 2 | Odd << 1 | IQ) << 24) | (seq & 0xffffff);

    return tag;

    std::cout << "format: " << format << " chan:" << chan << " odd:" << odd << " Q:" << Q << std::endl;
    std::cout.width(8);
    std::cout.fill('0');
    std::cout << std::hex << tag <<std::endl;
    return tag;
}

//////////////////////////////////////////////////////////////////////////////////
void
p7142sd3cDn::ciDecodeTag(uint32_t tag, int& format, int& chan, bool& odd, bool& Q, uint32_t& seq) {
    /// The CI tag, in little endian format as described in VHDL:
    ///  --! bits 31:28  Format number   0-15(4 bits)
    ///  --! bits 27:26  Channel number  0-3 (2 bits)
    ///  --! bits    25  0=even, 1=odd   0-1 (1 bit)
    ///  --! bit     24  0=I, 1=Q        0-1 (1 bit)
    ///  --! bits 23:00  Sequence number     (24 bits)

    format =        (tag >> 28) & 0xf;
    chan   =        (tag >> 26) & 0x3;
    odd    = (bool) ((tag >> 25) & 0x1);
    Q      = (bool) ((tag >> 24) & 0x1);
    seq    =        (tag & 0xffffff);

    return;

    std::cout << "decoded format: " << format << " chan:"
            << chan << " odd:" << odd << " Q:" << Q
            << " seq:" << seq << std::endl;
    std::cout.width(8);
    std::cout.fill('0');
    std::cout << "decoded tag:" << std::hex << tag << std::dec << std::endl;
    return;
}

//////////////////////////////////////////////////////////////////////////////////
void
p7142sd3cDn::initBuffer() {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    // note that _beamLength is only the length of the
    // IQ data (in bytes).

    switch(_sd3c._operatingMode()) {
    case p7142sd3c::MODE_FREERUN:
        // free run mode has:
        //   16 bit I and Q pairs for each gate
        _beamLength = _gates * 2 * 2;
      break;
    case p7142sd3c::MODE_PULSETAG:
        // pulse tag mode has:
        //    16 bit I and Q pairs for each gate
        _beamLength = _gates * 2 * 2;
        break;
    case p7142sd3c::MODE_CI:
        // coherent integration mode has:
        //   even 32 bit I and Q pairs followed by
        //   odd  32 bit I and Q pairs,
        // for each gate.
        _beamLength = _gates * 2 * 2 * 4;
        break;
    default:
        std::cerr << __PRETTY_FUNCTION__ << ": unknown SD3C mode: " << 
            _sd3c._operatingMode() << std::endl;
        abort();
    }

    // allocate the buffer to hold one (or two for CI) beams of IQ data
    _buf = new char[_beamLength];

}

//////////////////////////////////////////////////////////////////////////////////
void
p7142sd3cDn::makeSimData(int n) {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    int r;

    while(_simFifo.size() < (unsigned int)n) {
        switch(_sd3c._operatingMode()) {
        case p7142sd3c::MODE_FREERUN: {
            // ************* free run mode ***************
            for (int i = 0; i < _beamLength/4; i++) {
                uint32_t iq;
                char* p = (char*)&iq;
                r = p7142Dn::read(p, 4);
                assert(r == 4);
                for (int j = 0; j < 4; j++) {
                    _simFifo.push_back(p[j]);
                }
            }
            break;
        }
        case p7142sd3c::MODE_PULSETAG: {
            // ********** pulse tag mode **************
            // Add sync word
            uint32_t syncword = SD3C_SYNCWORD;
            for (int i = 0; i < 4; i++) {
                _simFifo.push_back(((char*)&syncword)[i]);
            }
            // Add the pulse tag for this sample:
            //       bits 31:30  Channel number         0-3 (2 bits)
            //       bits 29:00  Pulse sequence number  0-1073741823 (30 bits)
            // This is packed as a little-endian order 4-byte word;
            uint32_t tag = (_chanId << 30) | (_simPulseNum & 0x3fffffff);
            char* p = (char*)&tag;
            for (int i = 0; i < 4; i++) {
                _simFifo.push_back(p[i]);
            }
            // Add IQ data. Occasionally drop some data
            bool doBadSync = ((1.0 * rand())/RAND_MAX) < 5.0e-6;
            doBadSync = false;
            int nPairs = _beamLength/4;
            if (doBadSync) {
                nPairs = (int)(((1.0 * rand())/RAND_MAX) * nPairs);
            }
            for (int i = 0; i < nPairs; i++) {
                uint32_t iq;
                char* p = (char*)&iq;
                r = p7142Dn::read(p, 4);
                assert(r == 4);
                for (int j = 0; j < 4; j++) {
                    _simFifo.push_back(p[j]);
                }
            }
            _simPulseNum++;
            if (_simPulseNum > MAX_PT_PULSE_NUM) {
                _simPulseNum = 0;
            }
            break;
        }
        case p7142sd3c::MODE_CI: {
            /// Add the coherent integration tag for this sample:

            /// --! <TAG_I_EVEN><TAG_Q_EVEN><TAG_I_ODD><TAG_Q_ODD><IQpairs,even pulse><IQpairs,odd pulse>
            ///
            ///  --! bits 31:28  Format number   0-15(4 bits)
            ///  --! bits 27:26  Channel number  0-3 (2 bits)
            ///  --! bits    25  0=even, 1=odd   0-1 (1 bit)
            ///  --! bit     24  0=I, 1=Q        0-1 (1 bit)
            ///  --! bits 23:00  Sequence number     (24 bits)

            for (int j = 0; j < 4; j++) {
                uint32_t tag = ciMakeTag(1, _chanId, (j>>1)&1, j&1, _simPulseNum);
                char* p = (char*)&tag;
                for (int i = 0; i < 4; i++) {
                    _simFifo.push_back(p[i]);
                }
            }

            // Add IQ data. Occasionally drop some data
            bool doBadSync = ((1.0 * rand())/RAND_MAX) < 5.0e-6;
            doBadSync = false;
            int nPairs = _beamLength/8;
            if (doBadSync) {
                nPairs = (int)(((1.0 * rand())/RAND_MAX) * nPairs);
            }
            for (int i = 0; i < nPairs; i++) {
                char iq[8];
                r = p7142Dn::read(iq, 8);
                assert(r == 8);
                for (int j = 0; j < 8; j++) {
                    _simFifo.push_back(iq[j]);
                }
            }
            _simPulseNum += 1;
            if (_simPulseNum > MAX_CI_PULSE_NUM) {
                _simPulseNum = 0;
            }

            break;
        }
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////
void
p7142sd3cDn::simWait() {
    // because the usleep overhead is large, sleep every 100 calls
   	if (_simPauseMS > 0) {
   	    if (!(_simWaitCounter++ % 100)) {
        	usleep((int)(100*_simPauseMS*1000)*_nsum);
    	}
    }
}
//////////////////////////////////////////////////////////////////////////////////
void
p7142sd3cDn::unpackPtChannelAndPulse(const char* buf, unsigned int & chan,
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
p7142sd3cDn::droppedPulses() {
    boost::recursive_mutex::scoped_lock guard(_mutex);
    unsigned long retval = _droppedPulses;
    return retval;
}

//////////////////////////////////////////////////////////////////////////////////
unsigned long
p7142sd3cDn::syncErrors() {
    boost::recursive_mutex::scoped_lock guard(_mutex);
    unsigned long retval = _syncErrors;
    return retval;
}

//////////////////////////////////////////////////////////////////////////////////
void
p7142sd3cDn::dumpSimFifo(std::string label, int n) {
    boost::recursive_mutex::scoped_lock guard(_mutex);
    std::cout << label <<  " _simFifo length: " << _simFifo.size() << std::endl;
    std::cout << std::hex;
    for (unsigned int i = 0; i < (unsigned int)n && i < _simFifo.size(); i++) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)(unsigned char)_simFifo[i] << " ";
        if (!((i+1) % 40)) {
            std::cout << std::endl;
        }
    }
    std::cout << std::dec << std::endl;;
}

} // end namespace Pentek
