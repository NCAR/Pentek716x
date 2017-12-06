#include "BuiltinGaussian.h"
#include "BuiltinKaiser.h"
#include "FilterSpec.h"
#include <sys/ioctl.h>
#include <cerrno>
#include <cmath>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <716x.h>
#include <logx/Logging.h>

#include "p716x_sd3c.h"
#include "p716xDn_sd3c.h"
LOGGING("p716xDn_sd3c")

using namespace boost::posix_time;

namespace Pentek {

  ////////////////////////////////////////////////////////////////////////////////
  p716xDn_sd3c::p716xDn_sd3c(p716x_sd3c * p716xSd3cPtr, 
                             int chanId, 
                             uint32_t dmaDescSize,
                             bool isBurst,
                             int tsLength,
                             double rx_delay,
                             double rx_pulsewidth,
                             std::string gaussianFile,
                             std::string kaiserFile,
                             int simWaveLength,
                             bool internalClock) :
    p716xDn(p716xSd3cPtr, 
            chanId, 
            dmaDescSize,
            1, 
            simWaveLength,
            p716xSd3cPtr->nsum() > 1,
            internalClock),
    _sd3c(*p716xSd3cPtr),
    _isBurst(isBurst),
    _decimation(1),
    _tsLength(tsLength),
    _gaussianFile(gaussianFile), 
    _kaiserFile(kaiserFile),
    _lastPulse(0),
    _nPulsesSinceStart(0),
    _droppedPulses(0),
    _syncErrors(0),
    _firstRawBeam(true),
    _firstBeam(true),
    _dataInterruptPeriod(0.0),
    _timeLastSyncErrorPrint(0)

  {

    boost::recursive_mutex::scoped_lock guard(_mutex);

    // Update the default ADC configuration with our local modifications
    _initSd3cAdc();

    // Get gate count and coherent integration sum count from our card
    _gates = _sd3c.gates();
    _nsum = _sd3c.nsum();
    
    // Convert our rx delay and width to counts.
    int rxDelayCounts = _sd3c.timeToCounts(rx_delay);
    int rxPulsewidthCounts = _sd3c.timeToCounts(rx_pulsewidth);

    // log startup params in debug mode

    DLOG << "+++++++++++++++++++++++++++++";
    DLOG << "p716xSd3cDn constructor";
    DLOG << "           cardIndex: " << _p716x.getCardIndex();
    DLOG << "              chanId: " << chanId;
    DLOG << "             isBurst: " << isBurst;
    DLOG << "            tsLength: " << tsLength;
    DLOG << "            rx_delay: " << rx_delay;
    DLOG << "       rx_pulsewidth: " << rx_pulsewidth;
    DLOG << "        gaussianFile: " << gaussianFile;
    DLOG << "          kaiserFile: " << kaiserFile;
    DLOG << "       simWaveLength: " << simWaveLength;
    DLOG << "       internalClock: " << internalClock;
    DLOG << "               gates: " << _gates;
    DLOG << "                nsum: " << _nsum;
    DLOG << "  rxPulsewidthCounts: " << rxPulsewidthCounts;
    DLOG << "       ddcDecimation: " << _sd3c.ddcDecimation();
    DLOG << "        adcFrequency: " << _sd3c.adcFrequency();
    DLOG << "++++++++++++++++++++++++++++";

    // Make sure the rx_pulsewidth is a multiple of the time per decimated
    // sample.
    if (rxPulsewidthCounts == 0 ||
        ((2 * rxPulsewidthCounts) % _sd3c.ddcDecimation()) != 0) {
      ELOG << "Chan " << chanId << " rx_pulsewidth (digitizer_sample_width) must be a " <<
        "non-zero multiple of " <<
        1.0e9 * _sd3c.ddcDecimation() / _sd3c.adcFrequency() <<
        " ns for " << _sd3c.ddcTypeName();
      raise(SIGINT);
    }
    
    // PRT must be a multiple of the pulse width and longer than
    // (gates + 1) * pulse width
    if (!_isBurst) {
      if ((_sd3c.prtCounts() % rxPulsewidthCounts)) {
        ELOG << "Chan " << chanId << " rx pulse width must divide into PRT";
        ELOG << "rxPulsewidthCounts, prtCounts: "
             << rxPulsewidthCounts << ", " << _sd3c.prtCounts();
        raise(SIGINT);
      }
      if (_sd3c.prtCounts() <= ((_sd3c.gates() + 1) * rxPulsewidthCounts)) {
        ELOG << "Chan " << chanId << "PRT ERROR";
        ELOG << "PRT: " << _sd3c.prt() << " sec, "
             <<  _sd3c.prtCounts() << " counts";
        ELOG << "rx pulse width: " 
             << _sd3c.countsToTime(rxPulsewidthCounts) << " sec, "
             << rxPulsewidthCounts << " counts";
        ELOG << "n gates: " << _sd3c.gates();
        ELOG << "rx pulse width: " << rx_pulsewidth;
        ELOG << "PRT must be greater than (gates+1)*(rx pulse width)";
        ELOG << "Min valid PRT: " << ((_sd3c.gates()+1) * rx_pulsewidth);
        raise(SIGINT);
      }
    }

    // DDCx-specific testing for channel type.
    switch (_sd3c.ddcType()) {
      // DDC8 and DDC10 support burst sampling on channel 2
      case p716x_sd3c::DDC8DECIMATE:
      case p716x_sd3c::DDC10DECIMATE:
        if (_isBurst) {
          if (_chanId < 2) {
            ELOG << "Burst requested, channel: " << _chanId;
            ELOG << "Burst only support on channel 2";
            raise(SIGINT);
          }
        } else {
          if (_chanId > 1) {
            ELOG << "Normal sampling requested, Channel: " << _chanId;
            ELOG << "Normal sampling only supported on 0 or 1";
            raise(SIGINT);
          }
        }
        break;
        // Other DDC-s do not support burst sampling
      default:
        // Burst not supported for non-burst DDC-s
        if (_isBurst) {
          ELOG << "Burst requested, channel: " << _chanId;
          ELOG << "But not supported on: " << p716x_sd3c::ddcTypeName(_sd3c.ddcType());
          raise(SIGINT);
        }
        break;
    }

    // Which RX gating timer for this channel? Channels 0 and 1 share
    // p716x_sd3c::RX_01_TIMER, and channel 2 uses p716x_sd3c::RX_2_TIMER.
    p716x_sd3c::TimerIndex gatingTimer = (_chanId < 2) ?
      p716x_sd3c::RX_01_TIMER : p716x_sd3c::RX_2_TIMER;

    // Set the rx gating timer.
    /// @todo This really should not be done in p716xDn, because the
    /// timers are global resources; they don't necessarily belong to a
    /// particular channel.
    if (_isBurst) {
      // Burst channel. The rxPulsewidth gives the total sampling time for
      // the burst, and a "gate" is generated for each sample clock cycle.
      _gates = rxPulsewidthCounts;
      _sd3c._setTimer(gatingTimer, rxDelayCounts, rxPulsewidthCounts);
    } else {
      // rx gating for a normal downconversion channel
      _sd3c._setTimer(gatingTimer, rxDelayCounts, rxPulsewidthCounts * _gates);
    }
    
    /// Estimate the period between data-available interrupts based on the
    /// configured DMA transfer length. This is a good first-order estimate
    /// of maximum data latency time for the channel.
    
    double chanDataRate = 
      (2 * sizeof(uint32_t) * _gates) / _sd3c.prt();   /// @TODO this only works for single PRT
    _dataInterruptPeriod = _DmaDescSize / chanDataRate;
    if (p716xSd3cPtr->nsum() > 1) {
      _dataInterruptPeriod /= (p716xSd3cPtr->nsum() / 2);
    }

    // Warn if data latency is greater than 1 second, and bail completely if
    // it's greater than 5 seconds.
    if (_dataInterruptPeriod > 1.0) {
      ELOG << "DMA transfer size " << _DmaDescSize << " bytes";
      ELOG << "chanDataRate " << chanDataRate << " bytes/s";
      ELOG << "Warning: Estimated max data latency for channel " << 
        _chanId << " is " << _dataInterruptPeriod << " s!";
    }
    if (_dataInterruptPeriod > 5.0) {
      ELOG << "Calculated data interrupt period is greater than 5 seconds!";
      raise(SIGINT);
    }
    
    // initialize the buffering scheme.
    initBuffer();

    if (isSimulating()) {
      return;
    }

    // Set the decimation for this receive channel
    // ** This establishes the gate width in the downconverter. **
    // It must be the number of counts of the ADC_CLK10D
    // _isBurst ? _setDecimation(1) : _setDecimation(rxPulsewidthCounts/5);
    _isBurst ? _setDecimation(1) : _setDecimation(rxPulsewidthCounts/4);
    
    // configure DDC in FPGA
    if (!config()) {
      DLOG << "error initializing filters";
    }

  }

  ////////////////////////////////////////////////////////////////////////////////
  p716xDn_sd3c::~p716xDn_sd3c() {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    // Put SD3C timers into reset
    ILOG << "Channel " << _chanId << " destructor: putting timers in reset";
    _sd3c.setTimersResetBit();

    delete [] _buf;
    ILOG << "ADC channel " << _chanId << " finished shutting down.";
  }

  ////////////////////////////////////////////////////////////////////////////////
  void
    p716xDn_sd3c::_setDecimation(uint16_t decimation) {

    _decimation = decimation;

    // Write to the SD3C decimation register
    void * regAddr = _sd3c._sd3cRegAddr(DDC_DECIMATION1 + _chanId);
    P716x_REG_WRITE(regAddr, _decimation);

    // Read back to verify writing
    uint32_t readback;
    P716x_REG_READ(regAddr, readback);
    if (readback != _decimation) {
      ELOG << "Failed to write SD3C decimation register for channel " << _chanId;
      raise(SIGINT);
    }
    DLOG << "Channel " << _chanId << " decimation: " << _decimation;

  }

  ////////////////////////////////////////////////////////////////////////////////
  void p716xDn_sd3c::_initSd3cAdc() {

    // Have the ADC put its data into the "user block", so that it will go 
    // to our DDC running on the FPGA before being packed for output.
    _adcParams.dataSelect = P716x_ADC_DATA_CTRL_USR_DATA_SEL_USER;

    // Use the USER_DVAL (user data valid) signal from the user block to trigger
    // delivery of data to the ADC PACK FIFO.
    _adcParams.userDataValidEnable = P716x_ADC_GATE_TRIG_CTRL_USER_DVAL_ENABLE;

    // Set ADC data mode to deliver 32-bit I and Q data
    _adcParams.dataPackMode = P716x_ADC_DATA_CTRL_PACK_MODE_IQ_DATA_UNPACK;

    // Use gate mode for enabling writes to the ADC PACK FIFO
    _adcParams.triggerMode = P716x_ADC_GATE_TRIG_CTRL_TRIG_MODE_GATE;

    // Enable writing data to the ADC PACK FIFO when the gate signal is high
    _adcParams.gateTrigEnable = P716x_ADC_GATE_TRIG_CTRL_GATE_TRIG_IN_ENABLE;

    // Enable the RAM fifo.
    _adcParams.ramPathEnable = P716x_ADC_RAM_CTRL_RAM_PATH_ENABLE;
    _adcParams.ramReadEnable = P716x_ADC_RAM_CTRL_RAM_READ_ENABLE;

    // Apply the new configuration to the ADC registers
    _applyAdcParams();
  }

  ////////////////////////////////////////////////////////////////////////////////
  std::string p716xDn_sd3c::ddcTypeName() const {
    return p716x_sd3c::ddcTypeName(_sd3c.ddcType());
  }

  ////////////////////////////////////////////////////////////////////////////////
  double p716xDn_sd3c::rcvrPulseWidth() const {
    // Note that Channels 0 and 1 share RX_01_TIMER, and channel 2 uses
    // RX_2_TIMER.
    p716x_sd3c::TimerIndex rxTimerNdx = (_chanId <= 1) ? 
      p716x_sd3c::RX_01_TIMER : p716x_sd3c::RX_2_TIMER;
    return(_sd3c.countsToTime(_sd3c._timerWidth(rxTimerNdx)) / _gates);
  }

  ////////////////////////////////////////////////////////////////////////////////
  double p716xDn_sd3c::rcvrFirstGateDelay() const {
    int txDelayCounts = _sd3c._timerDelay(p716x_sd3c::TX_PULSE_TIMER);
    // Note that Channels 0 and 1 share RX_01_TIMER, and channel 2 uses
    // RX_2_TIMER.
    p716x_sd3c::TimerIndex rxTimerNdx = (_chanId <= 1) ? 
      p716x_sd3c::RX_01_TIMER : p716x_sd3c::RX_2_TIMER;
    int rxDelayCounts = _sd3c._timerDelay(rxTimerNdx);

    return(_sd3c.countsToTime(rxDelayCounts - txDelayCounts));
  }

  ////////////////////////////////////////////////////////////////////////////////
  double p716xDn_sd3c::gateSpacing() const {
    return(0.5 * SPEED_OF_LIGHT * rcvrPulseWidth());
  }

  ////////////////////////////////////////////////////////////////////////////////
  bool p716xDn_sd3c::config() {

    boost::recursive_mutex::scoped_lock guard(_mutex);

    // configure the fifo
    fifoConfig();

    // Is coherent integration enabled?
    DLOG << "coherent integration is " <<
      (_nsum > 1 ? "enabled" : "disabled");
    
    // set up the filters. Will do nothing if either of
    // the filter file paths is empty or if this is a burst channel
    // load the filter coefficients

    if (_gaussianFile.find("no-filter") != std::string::npos) {
      DLOG << "Found 'no-filter', no gaussian filter loaded\n";
    } else {
      FilterSpec gaussian;
      if (gaussianFilterSetup(gaussian)) {
        return false;
      }
      if (!loadGaussianFilter(gaussian)) {
        ELOG << "Unable to load gaussian filter\n";
        if (! _p716x.usingInternalClock()) {
          ELOG << "Is the external clock source connected?";
          ELOG << "Is the clock signal strength at least +3 dBm?";
        }
        exit(1);
      }
    }
    
    if (_kaiserFile.find("no-filter") != std::string::npos) {
      DLOG << "Found 'no-filter', no kaiser filter loaded\n";
    } else {
      FilterSpec kaiser;
      if (kaiserFilterSetup(kaiser)) {
        return false;
      }
      if (!loadKaiserFilter(kaiser)) {
        ELOG << "Unable to load kaiser filter\n";
        if (! _p716x.usingInternalClock()) {
          ELOG << "Is the external clock source connected?";
          ELOG << "Is the clock signal strength at least +3 dBm?";
        }
        exit(1);
      }
    }
    
    return true;
  }
  
  //////////////////////////////////////////////////////////////////////
  bool p716xDn_sd3c::loadKaiserFilter(FilterSpec& kaiser) {

    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (isSimulating()) {
      return true;
    }
    
    int ddcSelect = _chanId << 14;

    // program the kaiser coefficients

    bool kaiserFailed = false;
    std::ostringstream stream;
    for (unsigned int i = 0; i < kaiser.size(); i++) {

      // Set up to write this coefficient
      int ramAddr = 0;
      int ramSelect = 0;
      switch (_sd3c.ddcType()) {
        case p716x_sd3c::DDC10DECIMATE:
          ramAddr = i / 10;
          ramSelect = (i % 10) << 4;
          break;
        case p716x_sd3c::DDC8DECIMATE:
          ramAddr = i / 8;
          ramSelect = (i % 8) << 4;
          break;
        case p716x_sd3c::DDC4DECIMATE:
          ramAddr = i / 4;
          ramSelect = (i % 4) << 4;
          break;
        case p716x_sd3c::BURST:   // Burst mode uses no filters
          break;    
      }
        
      P716x_REG_WRITE(_sd3c._sd3cRegAddr(KAISER_CTL),
                      ddcSelect | DDC_STOP | ramSelect | ramAddr);
      usleep(1);

      // Try up to a few times to program this filter coefficient and
      // read it back successfully.
      bool coeffLoaded = false;
      for (int attempt = 0; attempt < 5; attempt++) {
        // write the value
        // LS word first
        P716x_REG_WRITE(_sd3c._sd3cRegAddr(KAISER_DATA_LSW), kaiser[i] & 0xFFFF);
        usleep(1);
    
        // then the MS word -- since coefficients are 18 bits and FPGA 
        // registers are 16 bits!
        P716x_REG_WRITE(_sd3c._sd3cRegAddr(KAISER_DATA_MSW),
                        (kaiser[i] >> 16) & 0x3);
        usleep(1);
    
        // latch coefficient
        P716x_REG_WRITE(_sd3c._sd3cRegAddr(KAISER_WR), 0x1);
        usleep(1);
    
        // disable writing (kaiser readback only succeeds if we do this)
        P716x_REG_WRITE(_sd3c._sd3cRegAddr(KAISER_WR), 0x0);
        usleep(1);
    
        // read back the programmed value; we need to do this in two words 
        // as above.
        unsigned int readBack;
        uint32_t kaiser_lsw;
        uint32_t kaiser_msw;
        P716x_REG_READ(_sd3c._sd3cRegAddr(KAISER_READ_LSW), kaiser_lsw);
        P716x_REG_READ(_sd3c._sd3cRegAddr(KAISER_READ_MSW), kaiser_msw);
        readBack = kaiser_msw << 16 | kaiser_lsw;

        if (readBack == kaiser[i]) {
          coeffLoaded = true;
          if (attempt != 0) {
            DLOG << ":" << std::hex << readBack << std::dec <<
              " -- OK";
          }
          break;
        } else {
          if (attempt == 0) {
            stream.str().clear();
            stream << "kaiser[" << i << "] = " << std::hex <<
              kaiser[i] << ", readbacks: " << readBack <<
              std::dec;
          } else {
            stream << ":" << std::hex << readBack << std::dec;
          }
        }
      }
      if (! coeffLoaded) {
        stream << " -- FAILED!";
        DLOG << stream.str();
      }
        
      kaiserFailed |= !coeffLoaded;
    }

    if (!kaiserFailed) {
      DLOG << kaiser.size()
           << " Kaiser filter coefficients successfully loaded: "
           << kaiser.name();
    } else {
      DLOG << "Unable to load the Kaiser filter coefficients";
      DLOG << kaiser.toStr();
    }

    // return to decimal output
    DLOG << std::dec;

    return !kaiserFailed;

  }

  //////////////////////////////////////////////////////////////////////
  bool p716xDn_sd3c::loadGaussianFilter(FilterSpec& gaussian) {

    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (isSimulating()) {
      return true;
    }
    
    std::ostringstream stream;
    int ddcSelect = _chanId << 14;

    // program the gaussian coefficients
    // Note that the DDC select is accomplished in the kaiser filter coefficient
    // address register, which was done during the previous kaiser filter load.

    bool gaussianFailed = false;
    for (unsigned int i = 0; i < gaussian.size(); i++) {

      // Set up to write this coefficient
      int ramAddr = 0;
      int ramSelect = 0;
      switch (_sd3c.ddcType()) {
        case p716x_sd3c::DDC10DECIMATE:
          ramAddr = i % 10;
          ramSelect = (i / 10) << 4;
          break;
        case p716x_sd3c::DDC8DECIMATE:
          ramAddr = i % 8;
          ramSelect = (i / 8) << 4;
          break;    
        case p716x_sd3c::DDC4DECIMATE:
          ramAddr = i % 12;
          ramSelect = (i / 12) << 4;
          break;
        case p716x_sd3c::BURST:   // Burst mode uses no filters
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
        P716x_REG_WRITE(_sd3c._sd3cRegAddr(GAUSSIAN_CTL),
                        ddcSelect | ramSelect | ramAddr);
        usleep(1);
    
        // write the value
        // LS word first
        P716x_REG_WRITE(_sd3c._sd3cRegAddr(GAUSSIAN_DATA_LSW),
                        gaussian[i] & 0xFFFF);
        usleep(1);
    
        // then the MS word -- since coefficients are 18 bits and FPGA 
        // registers are 16 bits!
        P716x_REG_WRITE(_sd3c._sd3cRegAddr(GAUSSIAN_DATA_MSW),
                        (gaussian[i] >> 16) & 0x3);
        usleep(1);
    
        // latch coefficient
        P716x_REG_WRITE(_sd3c._sd3cRegAddr(GAUSSIAN_WR), 0x1);
        usleep(1);
    
        // disable writing (gaussian readback only succeeds if we do this)
        P716x_REG_WRITE(_sd3c._sd3cRegAddr(GAUSSIAN_WR), 0x0);
        usleep(1);
    
        // read back the programmed value; we need to do this in two words 
        // as above.
        unsigned int readBack;
        uint32_t kaiser_lsw;
        uint32_t kaiser_msw;
        P716x_REG_READ(_sd3c._sd3cRegAddr(GAUSSIAN_READ_LSW), kaiser_lsw);
        P716x_REG_READ(_sd3c._sd3cRegAddr(GAUSSIAN_READ_MSW), kaiser_msw);
        readBack = kaiser_msw << 16 | kaiser_lsw;
        if (readBack == gaussian[i]) {
          coeffLoaded = true;
          if (attempt != 0) {
            DLOG << ":" << std::hex << readBack << std::dec <<
              " -- OK";
          }
          break;
        } else {
          if (attempt == 0) {
            stream.str().clear();
            stream << "gaussian[" << i << "] = " << std::hex <<
              gaussian[i] << ", readbacks: " << readBack <<
              std::dec;
          } else {
            stream << ":" << std::hex << readBack << std::dec;
          }
        }
      }
      if (! coeffLoaded) {
        stream << " -- FAILED!";
        DLOG << stream.str();
      }
        
      gaussianFailed |= !coeffLoaded;
    }

    if (!gaussianFailed) {
      DLOG << gaussian.size()
           << " Gaussian filter coefficients successfully loaded: "
           << gaussian.name();
    } else {
      DLOG << "Unable to load the Gaussian filter coefficients";
      DLOG << gaussian.toStr();
    }

    // return to decimal output
    DLOG << std::dec;

    return gaussianFailed;

  }

  ////////////////////////////////////////////////////////////////////////
  // set up gaussian filter
  // returns 0 on success, -1 on failure

  int p716xDn_sd3c::gaussianFilterSetup(FilterSpec &filterSpec) {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    // No filters if this is a burst sampling channel
    if (_isBurst) {
      return 0;
    }

    // get the gaussian filter coefficients.
    if (_gaussianFile.size() != 0) {
      FilterSpec g(_gaussianFile);
      if (!g.ok()) {
        ELOG << "Incorrect or unaccessible filter definition: "
             << _gaussianFile;
        return -1;
      } else {
        filterSpec = g;
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
        case p716x_sd3c::DDC8DECIMATE: {
          switch ((int)(_sd3c.countsToTime(_sd3c._timerWidth(p716x_sd3c::TX_PULSE_TIMER)) * 1.0e7)) {

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
              ELOG << "chip width specification of "
                   << _sd3c._timerWidth(p716x_sd3c::TX_PULSE_TIMER)
                   << " is not recognized, filter will be configured for a "
                   << pulsewidthUs << " uS pulse\n";
              break;
          }
          break;
        }
        case p716x_sd3c::DDC4DECIMATE: {

          // Figure out the filter bandwidth, in nanoseconds
          int fwidth_ns =
            (int)(1.0e9 *
                  _sd3c.countsToTime(_sd3c._timerWidth(p716x_sd3c::TX_PULSE_TIMER)) /
                  _sd3c.codeLength());

          // Find the gaussian filter coefficient set corresponding to this filter width
          switch (fwidth_ns) {
            case 500:
              // 0.5 uS (75m gate)
              gaussianFilterName = "ddc4_0_500";
              break;
            case 666:
              // 0.667 uS (100m gate)
              gaussianFilterName = "ddc4_0_667";
              break;
            case 1000:
              // 1.0 uS (150m gate)
              gaussianFilterName = "ddc4_1_000";
              break;
            case 1333:
              // 1.333 uS (200m gate)
              gaussianFilterName = "ddc4_1_333";
              break;
            case 2666:
              // 2.667 uS (400m gate)
              gaussianFilterName = "ddc4_2_667";
              break;
            default:
              ELOG << "chip width specification of "
                   << _sd3c._timerWidth(p716x_sd3c::TX_PULSE_TIMER)/_sd3c.codeLength()
                   << " (" 
                   << (_sd3c.countsToTime(_sd3c._timerWidth(p716x_sd3c::TX_PULSE_TIMER)) /
                       _sd3c.codeLength())
                   << "s) is not recognized\n";
              raise(SIGINT);
              break;
          }
          if (_sd3c.codeLength() > 1) {
            // If we are using pulse coding, choose the gaussian filter variant for that.
            gaussianFilterName += "_pulsecode";
          }

          break;
        }
        case p716x_sd3c::DDC10DECIMATE: {
          // pulse_width in 50 MHz counts
          pulsewidthUs = 0.5;
          gaussianFilterName = "ddc10_0_5_flat";
          break;
        }
        default: {
          ELOG << "DDC type " << ddcTypeName() << 
            " not handled in " << __FUNCTION__;
          raise(SIGINT);
        }
      }

      if (builtins.find(gaussianFilterName) == builtins.end()) {
        ELOG << "No entry for " << gaussianFilterName << ", "
             << pulsewidthUs
             << " us pulsewidth in the list of builtin Gaussian filters!";
        raise(SIGINT);
      }
      filterSpec = FilterSpec(builtins[gaussianFilterName]);
      DLOG << "Using gaussian filter coefficient set "
           << gaussianFilterName;
    }

    return 0;

  }

  ////////////////////////////////////////////////////////////////////////
  // set up kaiser filter
  // returns 0 on success, -1 on failure

  int p716xDn_sd3c::kaiserFilterSetup(FilterSpec &filterSpec) {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    // No filters if this is a burst sampling channel
    if (_isBurst) {
      return 0;
    }

    // get the kaiser filter coefficients
    std::string kaiserFilterName;
    if (_kaiserFile.size() != 0) {
      FilterSpec k(_kaiserFile);
      if (!k.ok()) {
        ELOG << "Incorrect or unaccessible filter definition: "
             << _kaiserFile;
        return -1;
      } else {
        filterSpec = k;
      }
    } else {
      BuiltinKaiser builtins;
      std::string kaiserFilterName;
      switch (_sd3c.ddcType()) {
        case p716x_sd3c::DDC8DECIMATE: {
          kaiserFilterName = "ddc8_5_0";
          break;
        }
        case p716x_sd3c::DDC4DECIMATE: {
          kaiserFilterName = "ddc4_4_0";
          break;
        }
        case p716x_sd3c::DDC10DECIMATE: {
          kaiserFilterName = "ddc10_5_0";
          break;
        }
        default: {
          ELOG << "DDC type " << ddcTypeName() << 
            " not handled in " << __FUNCTION__;
          raise(SIGINT);
        }
      }
      if (builtins.find(kaiserFilterName) == builtins.end()) {
        ELOG << "No entry for " << kaiserFilterName
             << " in the list of builtin Kaiser filters!";
        raise(SIGINT);
      }
      filterSpec = FilterSpec(builtins[kaiserFilterName]);
      DLOG << "Using kaiser filter coefficient set " << kaiserFilterName;
    }

    return 0;
  }

  //////////////////////////////////////////////////////////////////////
  void p716xDn_sd3c::fifoConfig() {

    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (isSimulating())
      return;

    // The FIFO for our channel needs to be configured

    // find the fifo configuration register
    unsigned int readBack;
    uint32_t fifoRegNum = ADC_FIFO_CTRL_1;
    switch (_chanId) {
      case 0:
        fifoRegNum = ADC_FIFO_CTRL_1;
        break;
      case 1:
        fifoRegNum = ADC_FIFO_CTRL_2;
        break;
      case 2:
        fifoRegNum = ADC_FIFO_CTRL_3;
        break;
      case 3:
        fifoRegNum = ADC_FIFO_CTRL_4;
        break;
    }

    P716x_REG_READ(_sd3c._sd3cRegAddr(fifoRegNum), readBack);

    // And configure ADC FIFO Control for this channel

    P716x_REG_WRITE(_sd3c._sd3cRegAddr(fifoRegNum), readBack & 0x000034BF);

  }

  //////////////////////////////////////////////////////////////////////////////////
  //
  // ******    Buffer management and data handling in the following section    *****
  //
  //////////////////////////////////////////////////////////////////////////////////

  //////////////////////////////////////////////////////////////////////
  int
    p716xDn_sd3c::_simulatedRead(char* buf, int n) {
    boost::recursive_mutex::scoped_lock guard(_mutex);

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
  /// get beam, decode the metadata before returning

  char*
    p716xDn_sd3c::getBeam(int64_t & nPulsesSinceStart,
                          double & angle1, double & angle2,
                          bool & xmitPolHorizontal) {

    // This method only works for pulse-tagged data
    if (_sd3c._operatingMode() != p716x_sd3c::MODE_PULSETAG) {
      ELOG << __PRETTY_FUNCTION__ << " only works for MODE_PULSETAG";
      raise(SIGINT);
    }

    // get beam with metadata

    uint32_t pulseMetadata[N_META_32];
    char *buf = ptBeamWithMeta(nPulsesSinceStart, pulseMetadata);

    // decode the metadata

    uint32_t pulseNum;
    uint32_t chanNum;
    unpackPtMetadata(pulseMetadata, pulseNum, chanNum,
                     angle1, angle2, xmitPolHorizontal);
    
    // return buffer

    return buf;

  }

  //////////////////////////////////////////////////////////////////////////////////
  /// get beam, with metadata

  char*
    p716xDn_sd3c::getBeam(int64_t & nPulsesSinceStart,
                          uint32_t pulseMetadata[N_META_32]) {
    // This method only works for pulse-tagged data
    if (_sd3c._operatingMode() != p716x_sd3c::MODE_PULSETAG) {
      ELOG << __PRETTY_FUNCTION__ << " only works for MODE_PULSETAG";
      raise(SIGINT);
    }
    return ptBeamWithMeta(nPulsesSinceStart, pulseMetadata);
  }

  //////////////////////////////////////////////////////////////////////////////////
  int
    p716xDn_sd3c::beamLength() {
    boost::recursive_mutex::scoped_lock guard(_mutex);
    return _beamLength;
  }

  //////////////////////////////////////////////////////////////////////////////////
  //
  // Get a beam, passing in a buffer for the metadata from the start of the beam
  // This will contain angles, scan flags etc that are interpreted by the
  // calling routine.
  // metaDataBuf must be allocated by the caller, to have the length bufLen,

  char*
    p716xDn_sd3c::ptBeamWithMeta(int64_t & nPulsesSinceStart,
                                 uint32_t pulseMetadata[N_META_32]) {
    
    boost::recursive_mutex::scoped_lock guard(_mutex);

    // get the beam, and fill the metadata array

    uint32_t pulseNum = 0;
    char* buf = ptBeam(pulseNum, pulseMetadata);

    // unpack the metadata
    
    uint32_t chanNum;
    double angle1, angle2;
    bool xmitPolHorizontal;
    unpackPtMetadata(pulseMetadata, pulseNum, chanNum,
                     angle1, angle2, xmitPolHorizontal);
    
    if (chanNum != _chanId) {
      if (nPulsesSinceStart % 10000 == 0) {
        ELOG << "==>> Bad chanNum: " << chanNum << ", should be: " << _chanId;
        ELOG << "====>> metadata[1]: " << std::hex 
             << "0x" << pulseMetadata[1] << std::dec;
        ELOG << "====>> pulseNum, HVflag: " << pulseNum << ", " << xmitPolHorizontal;
      }
    }
    
    // Initialize _lastPulse if this is the first pulse we've seen
    if (_firstBeam) {
      _lastPulse = pulseNum - 1;
      _firstBeam = false;
    }
    
    // Handle pulse number rollover gracefully
    if (_lastPulse == MAX_PT_PULSE_NUM) {
      DLOG << "Pulse number rollover on channel " << chanId();
      _lastPulse = -1;
    }

    // How many pulses since the last one we saw?
    int delta = pulseNum - _lastPulse;
    if (delta < (-MAX_PT_PULSE_NUM / 2)) {
      delta += MAX_PT_PULSE_NUM + 1;
    }
    
    if (delta == 0) {
      ELOG << "Channel " << _chanId << ": got repeat of pulse " <<
        pulseNum << "!";
      // raise(SIGINT);
    } else if (delta != 1) {
      ELOG << _lastPulse << "->" << pulseNum << ": ";
      if (delta < 0) {
        ELOG << "Channel " << _chanId << " went BACKWARD " <<
          -delta << " pulses";
      } else {
        ELOG << "Channel " << _chanId << " dropped " <<
          delta - 1 << " pulses";
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
    p716xDn_sd3c::ptBeam(uint32_t &pulseNum, uint32_t *metadata) 
  {

    boost::recursive_mutex::scoped_lock guard(_mutex);
    
    // How many sync errors at start?
    
    unsigned long startSyncErrors = _syncErrors;

    // check we are in sync
    
    checkInSync(pulseNum);
    
    // read in meta data
    
    int nToRead = N_META_32 * sizeof(uint32_t);
    int nread = read((char *) metadata, nToRead);
    assert(nread == nToRead);
    
    // ELOG << "====>> metadata words[0],[1]: " 
    //      << std::hex << std::setfill('0') << "0x" << std::setw(8)
    //      << metadata[0] << ", 0x" << std::setw(8) << metadata[1] << std::dec;
    // ELOG << "====>> metadata words[2],[3]: " 
    //      << std::hex << std::setfill('0') << "0x" << std::setw(8)
    //      << metadata[2] << ", 0x" << std::setw(8) << metadata[3] << std::dec;
    // ELOG << "====>> metadata words[4],[5]: " 
    //      << std::hex << std::setfill('0') << "0x" << std::setw(8)
    //      << metadata[4] << ", 0x" << std::setw(8) << metadata[5] << std::dec;

    // read in I/Q data
    
    nread = read(_buf, _beamLength);
    assert(nread == _beamLength);
    
    // uint32_t *iq = (uint32_t*) _buf;
    // for (int igate = 0; igate < _gates; igate++) {
    //   if (igate < 4 || igate >= (_gates - 4)) {
    //     ELOG << igate << ": "
    //          << std::hex << std::setfill('0')
    //          << "0x" << std::setw(8) << iq[igate*2]
    //          << ", 0x" << std::setw(8) << iq[igate*2+1] << std::dec;
    //   }
    // }

    if (_syncErrors != startSyncErrors) {
      time_t now = time(NULL);
      int secsSinceLastErrorPrint = now - _timeLastSyncErrorPrint;
      if (secsSinceLastErrorPrint > 1) {
        uint32_t * wordp = reinterpret_cast<uint32_t *>(pulseNum);
        ELOG << std::setfill('0');
        ELOG << "XX Got " << _syncErrors - startSyncErrors
             << " sync errors, cardIndex: " << _p716x.getCardIndex()
             << ", channel " << chanId();
        ELOG << " finding pulse w/tag 0x"
             << std::setw(8) << std::hex << *wordp
             << " after tag 0x" << std::setw(8)
             << (uint32_t(_chanId) << 30 | _lastPulse) << std::dec;
        _timeLastSyncErrorPrint = now;
      }
    }

    return _buf;
  }

  //////////////////////////////////////////////////////////////////////////////////
  // check we are in sync
  // returns 0 on success

  int p716xDn_sd3c::checkInSync(uint32_t pulseNum)

  {
  
    // read in next 2 32-bit words

    uint32_t sync[2];
    int nToRead = 2 * sizeof(uint32_t);
    int nread = read((char *) sync, nToRead);
    assert(nread == nToRead);

    // ELOG << "=======>> sync words [0],[1]: " 
    //      << std::hex << std::setfill('0')
    //      << "0x" << std::setw(8) << sync[0] << ", 0x" << std::setw(8) << sync[1]
    //      << std::dec;

    // check for sync

    if (sync[0] == SD3C_SYNCWORD_1 && sync[1] == SD3C_SYNCWORD_2) {
      // got sync
      return 0;
    }
    
    // need to resync

    std::ostringstream syncHuntMsg;
    syncHuntMsg << "Sync hunt on card: " << _p716x.getCardIndex()
                << ", chan: " << _chanId << " : ";
    syncHuntMsg << std::setfill('0');

    uint32_t nHuntWords = 0;
    while (true) {

      if (nHuntWords % 1000000 == 0) {
        ELOG << "ptBeam() syncing, chanId, nHuntWords: " << _chanId << ", " << nHuntWords;
      }
    
      // move latest word read
    
      sync[0] = sync[1];
    
      // read next word
    
      int nread = read((char *) &sync[1], sizeof(uint32_t));
      assert(nread == sizeof(uint32_t));
      nHuntWords++;
    
      // Break out when we've found sync
    
      if (sync[0] == SD3C_SYNCWORD_1 && sync[1] == SD3C_SYNCWORD_2) {
        break;
      }
    
    } // while (true)
        
    syncHuntMsg << "<SYNC>";
    time_t now = time(NULL);
    int secsSinceLastErrorPrint = now - _timeLastSyncErrorPrint;
    if (secsSinceLastErrorPrint > 1) {
      ELOG << "Sync hunt " << nHuntWords 
           << " words after pulse " << pulseNum
           << " on chan " << _chanId;
      DLOG << syncHuntMsg.str();
      _timeLastSyncErrorPrint = now;
    }

    return 0;

  }

  //////////////////////////////////////////////////////////////////////////////////
  char*
    p716xDn_sd3c::ciBeamDecoded(int64_t& nPulsesSinceStart, bool rim) {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    // get the beam
    unsigned int pulseNum;
    char* buf;
    if (!rim) { 
      buf = ciBeam(pulseNum);
    } else {
      buf = ciBeamRim(pulseNum);
    }

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
        DLOG << "Pulse number rollover";

      delta += MAX_CI_PULSE_NUM + 1;

    }

    if (delta == 0) {
      ELOG << "Channel " << _chanId << ": got repeat of pulse " <<
        pulseNum << "!";
      raise(SIGINT);
    } else if (delta != 1) {
      //ELOG << _lastPulse << "->" << pulseNum << ": ";
      if (delta < 0) {
        //ELOG << "Channel " << _chanId << " went BACKWARD " <<
        //    -delta << " pulses";
      } else {
        //ELOG << "Channel " << _chanId << " dropped " <<
        //    delta - 1 << " pulses";
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
    p716xDn_sd3c::ciBeam(unsigned int& pulseNum) {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    while(true) {
      if (_firstRawBeam) {
        // skip over the first 16 bytes, assuming that
        // they are a good tag word.
        int nread = read(_buf, 16);
        assert(nread == 16);
        _firstRawBeam = false;
      }

      // read one beam into buf
      {
        int nread = read(_buf, _beamLength);
        assert(nread == _beamLength);
      }

      // read the next tag word
      char tagbuf[16];
      {
        int nread = read(tagbuf, 16);
        assert(nread == 16);
        if (ciCheckTag(tagbuf, pulseNum)) {
          return _buf;
        }
        _syncErrors++;
      }
        
      // scan 4 bytes at a time for a correct tag
      while(true) {
        memmove(tagbuf, tagbuf+sizeof(uint32_t),12);
        int nread = read(tagbuf+12, sizeof(uint32_t));
        assert(nread == sizeof(uint32_t));
        // check for synchronization
        if (ciCheckTag(tagbuf, pulseNum)) {
          break;
        }
      }
    }

    return 0;
  }

  //////////////////////////////////////////////////////////////////////////////////
  char*
    p716xDn_sd3c::ciBeamRim(unsigned int& pulseNum) {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (0) {
      std::cout << "beam length is " << beamLength()
                << " (" << beamLength()/sizeof(uint32_t) << " I/Q words)" << std::endl;
      uint32_t bigbuf[2000];
      read((char*)bigbuf, 4*2000);
      for (int i = 0; i < 2000; i++) {
        if (!(i %16)) {
          std::cout << std::endl << std::setw(6)
                    << std::setfill(' ') << std::dec << i << " ";
        }
        std::cout << std::setw(8) << std::setfill('0')
                  << std::hex << bigbuf[i] << " ";
      }
      std::cout << std::dec;
    }

    while(true) {

      if (_firstRawBeam) {
        // skip over the first 64 bytes, assuming that
        // they are a good tag word.
        int nread = read(_buf, 64);
        assert(nread == 64);
        _firstRawBeam = false;
      }
      
      // read one beam into buf
      {
        int nread = read(_buf, _beamLength);
        assert(nread == _beamLength);
      }
      
      // read the next tag word
      char tagbuf[64];
      {
        int nread = read(tagbuf, 64);
        assert(nread == 64);
        if (ciCheckTagRim(tagbuf, pulseNum)) {
          return _buf;
        }
        _syncErrors++;
      }

      // scan 4 bytes at a time for a correct tag
      while(true) {
        memmove(tagbuf, tagbuf+4,12);
        int nread = read(tagbuf+12, sizeof(uint32_t));
        assert(nread == sizeof(uint32_t));
        // check for synchronization
        if (ciCheckTagRim(tagbuf, pulseNum)) {
          break;
        }
      }

    } // while

    return 0;
  }

  //////////////////////////////////////////////////////////////////////////////////
  char*
    p716xDn_sd3c::frBeam() {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    int nread = read(_buf, _beamLength);
    assert(nread == _beamLength);
    return _buf;
  }

  //////////////////////////////////////////////////////////////////////////////////
  bool
    p716xDn_sd3c::ciCheckTag(char* p, unsigned int& pulseNum) {

    // The tag and data order:
    //  (TAG_I_EVEN) (TAG_Q_EVEN) (TAG_I_ODD) (TAG_Q_ODD) (IQpairs,even_pulse) IQpairs,odd pulse
    //
    // The CI tag:
    //  bits 31:28  Format number   0-15(4 bits)
    //  bits 27:26  Channel number  0-3 (2 bits)
    //  bits    25  0=even, 1=odd   0-1 (1 bit)
    //  bit     24  0=I, 1=Q        0-1 (1 bit)
    //  bits 23:00  Sequence number     (24 bits)

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

    return retval;
  }

  //////////////////////////////////////////////////////////////////////////////////
  bool
    p716xDn_sd3c::ciCheckTagRim(char* p, unsigned int& pulseNum) {

    //  In range imaging mode, the tags are repeated four times (once per frequency)
    //
    // The tag and data order:
    // (TAG_I_EVEN) (TAG_Q_EVEN) (TAG_I_ODD) (TAG_Q_ODD)
    // (TAG_I_EVEN) (TAG_Q_EVEN) (TAG_I_ODD) (TAG_Q_ODD)
    // (TAG_I_EVEN) (TAG_Q_EVEN) (TAG_I_ODD) (TAG_Q_ODD)
    // (TAG_I_EVEN) (TAG_Q_EVEN) (TAG_I_ODD) (TAG_Q_ODD)
    // (IQpairs,even_pulse) IQpairs,odd pulse
    //
    // The CI tag:
    //  bits 31:28  Format number   0-15(4 bits)
    //  bits 27:26  Channel number  0-3 (2 bits)
    //  bits    25  0=even, 1=odd   0-1 (1 bit)
    //  bit     24  0=I, 1=Q        0-1 (1 bit)
    //  bits 23:00  Sequence number     (24 bits)

    int format[16];
    int chan[16];
    bool Odd[16];
    bool Q[16];
    uint32_t seq[16];
    for (int f = 0; f < 4; f++) {
      for (int i = 0; i < 4; i++) {
        uint32_t* tag = (uint32_t*)p;
        int index = 4*f + i;
        ciDecodeTag(tag[index], format[index], chan[index], Odd[index], Q[index], seq[index]);
      }
    }

    pulseNum = seq[0];

    // time to see if we received expected values
    bool retval = true;

    retval     = retval && (format[0] ==      2);

    for (int f = 0; f < 4; f++) {
      for (int i = 1; i < 4; i++) {
        retval = retval && (format[i+4*f] ==      2);
        retval = retval && (seq[i+4*f]    == seq[0]);
        retval = retval && (chan[i+4*f]   == chan[0]);
      }
      retval = retval && !Odd[0+4*f] && !Odd[1+4*f] && Odd[2+4*f] && Odd[3+4*f];
      retval = retval &&   !Q[0+4*f] &&    Q[1+4*f] &&  !Q[2+4*f] &&   Q[3+4*f];
    }

    return retval;
  }

  //////////////////////////////////////////////////////////////////////////////////
  uint32_t
    p716xDn_sd3c::ciMakeTag(int format, int chan, bool odd, bool Q, uint32_t seq) {
    /// The CI tag:
    ///  bits 31:28  Format number   0-15(4 bits)
    ///  bits 27:26  Channel number  0-3 (2 bits)
    ///  bits    25  0=even, 1=odd   0-1 (1 bit)
    ///  bit     24  0=I, 1=Q        0-1 (1 bit)
    ///  bits 23:00  Sequence number     (24 bits)

    int Odd = odd? 1:0;
    int IQ   =  Q? 1:0;
    uint32_t tag =
      (( format << 4 | chan << 2 | Odd << 1 | IQ) << 24) | (seq & 0xffffff);

    return tag;
    
    std::ostringstream stream;
    stream << "format: " << format << " chan:" << chan 
           << " odd:" << odd << " Q:" << Q;
    stream.width(8);
    stream.fill('0');
    stream << std::hex << tag;
    DLOG << stream.str();

    return tag;
  }

  //////////////////////////////////////////////////////////////////////////////////
  void
    p716xDn_sd3c::ciDecodeTag(uint32_t tag, int& format, int& chan, bool& odd, bool& Q, uint32_t& seq) {
    /// The CI tag, in little endian format as described in VHDL:
    ///  bits 31:28  Format number   0-15(4 bits)
    ///  bits 27:26  Channel number  0-3 (2 bits)
    ///  bits    25  0=even, 1=odd   0-1 (1 bit)
    ///  bit     24  0=I, 1=Q        0-1 (1 bit)
    ///  bits 23:00  Sequence number     (24 bits)

    format =        (tag >> 28) & 0xf;
    chan   =        (tag >> 26) & 0x3;
    odd    = (bool) ((tag >> 25) & 0x1);
    Q      = (bool) ((tag >> 24) & 0x1);
    seq    =        (tag & 0xffffff);

    return;

    std::ostringstream stream;
    stream << "decoded format: " << format << " chan:"
           << chan << " odd:" << odd << " Q:" << Q
           << " seq:" << seq;
    stream.width(8);
    stream.fill('0');
    stream << " decoded tag:" << std::hex << tag << std::dec;
    //DLOG << stream.str();
    std::cout << stream.str() << std::endl;

    return;
  }

  //////////////////////////////////////////////////////////////////////////////////
  void
    p716xDn_sd3c::initBuffer() {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    // note that _beamLength is only the length of the
    // IQ data (in bytes).

    switch(_sd3c._operatingMode()) {
      case p716x_sd3c::MODE_FREERUN:
        // free run mode has:
        //   16 bit I and Q pairs for each gate
        _beamLength = _gates * 2 * 2;
        break;
      case p716x_sd3c::MODE_PULSETAG:
        // pulse tag mode has:
        //    32 bit I and Q pairs for each gate
        _beamLength = _gates * 2 * sizeof(uint32_t);
        break;
      case p716x_sd3c::MODE_CI:
        // coherent integration mode has:
        //   even 32 bit I and Q pairs followed by
        //   odd  32 bit I and Q pairs,
        // for each gate.
        _beamLength = _gates * 2 * 2 * sizeof(uint32_t);
        break;
      case p716x_sd3c::MODE_CI_RIM:
        // RIM coherent integration mode has:
        //   even 32 bit I and Q pairs followed by
        //   odd  32 bit I and Q pairs,
        // for each gate,
    	// for 4 frequencies.
        _beamLength = 4*(_gates * 2 * 2 * sizeof(uint32_t));
        break;
      default:
        ELOG << __PRETTY_FUNCTION__ << ": unknown SD3C mode: " << 
          _sd3c._operatingMode();
        raise(SIGINT);
    }

    // allocate the buffer to hold one (or two for CI) beams of IQ data
    _buf = new char[_beamLength];

  }

  //////////////////////////////////////////////////////////////////////////////////
  void
    p716xDn_sd3c::makeSimData(int n) {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    while(_simFifo.size() < (unsigned int)n) {
      switch(_sd3c._operatingMode()) {
        case p716x_sd3c::MODE_FREERUN: {
          // ************* free run mode ***************
          for (size_t i = 0; i < _beamLength/sizeof(uint32_t); i++) {
            uint32_t iq;
            char* p = (char*)&iq;
            int nread = p716xDn::_simulatedRead(p, sizeof(uint32_t));
            assert(nread == sizeof(uint32_t));
            for (size_t j = 0; j < sizeof(uint32_t); j++) {
              _simFifo.push_back(p[j]);
            }
          }
          break;
        }
        case p716x_sd3c::MODE_PULSETAG: {
          // ********** pulse tag mode **************
          // Add sync word
          uint32_t syncword = SD3C_SYNCWORD_1;
          for (size_t i = 0; i < sizeof(uint32_t); i++) {
            _simFifo.push_back(((char*)&syncword)[i]);
          }
          // Add the pulse tag for this sample:
          //       bits 31:30  Channel number         0-3 (2 bits)
          //       bits 29:00  Pulse sequence number  0-1073741823 (30 bits)
          // This is packed as a little-endian order 4-byte word;
          uint32_t simPulseNum = _sd3c.nextSimPulseNum(_chanId);
          uint32_t tag = (_chanId << 30) | (simPulseNum & 0x3fffffff);
          char* p = (char*)&tag;
          for (size_t i = 0; i < sizeof(uint32_t); i++) {
            _simFifo.push_back(p[i]);
          }
          // Add all-zero metadata
          for (size_t ii = 0; ii < N_META_32 * sizeof(uint32_t); ii++) {
            _simFifo.push_back(0);
          }
          // Add IQ data. Occasionally drop some data
          bool doBadSync = ((1.0 * rand())/RAND_MAX) < 5.0e-6;
          doBadSync = false;
          int nPairs = _beamLength/sizeof(uint32_t);
          if (doBadSync) {
            nPairs = (int)(((1.0 * rand())/RAND_MAX) * nPairs);
          }
          for (int i = 0; i < nPairs; i++) {
            uint32_t iq;
            char* p = (char*)&iq;
            int nread = p716xDn::_simulatedRead(p, sizeof(uint32_t));
            assert(nread == sizeof(uint32_t));
            for (size_t j = 0; j < sizeof(uint32_t); j++) {
              _simFifo.push_back(p[j]);
            }
          }
          break;
        }
        case p716x_sd3c::MODE_CI: {
          /// Add the coherent integration tag for this sample:

          ///  (TAG_I_EVEN) (TAG_Q_EVEN) (TAG_I_ODD) (TAG_Q_ODD) (IQpairs,even_pulse) ((IQpairs,odd_pulse))
          ///
          ///  bits 31:28  Format number   0-15(4 bits)
          ///  bits 27:26  Channel number  0-3 (2 bits)
          ///  bits    25  0=even, 1=odd   0-1 (1 bit)
          ///  bit     24  0=I, 1=Q        0-1 (1 bit)
          ///  bits 23:00  Sequence number     (24 bits)

          uint32_t simPulseNum = _sd3c.nextSimPulseNum(_chanId);
          for (size_t j = 0; j < sizeof(uint32_t); j++) {
            //uint32_t tag = ciMakeTag(1, _chanId, (j>>1)&1, j&1, _simPulseNum);
            uint32_t tag = ciMakeTag(1, _chanId, (j>>1)&1, j&1, simPulseNum);
            char* p = (char*)&tag;
            for (size_t i = 0; i < sizeof(uint32_t); i++) {
              _simFifo.push_back(p[i]);
            }
          }

          // Add IQ data. Occasionally drop some data
          bool doBadSync = ((1.0 * rand())/RAND_MAX) < 5.0e-6;
          // Disable corrupted sync data for now.
          doBadSync = false;
          // I and Q values from the CI are 4 byte values,
          // so it will take 8 bytes for an I/Q pair.
          int nPairs = _beamLength/8;
          if (doBadSync) {
            nPairs = (int)(((1.0 * rand())/RAND_MAX) * nPairs);
          }
          for (int i = 0; i < nPairs; i++) {
            char iq[8];
            int nread = p716xDn::_simulatedRead(iq, 8);
            assert(nread == 8);
            for (int j = 0; j < 8; j++) {
              _simFifo.push_back(iq[j]);
            }
          }

          break;
        }
        case p716x_sd3c::MODE_CI_RIM: {
          /// Add the coherent integration (RIM) tag for this sample:

          ///  (TAG_I_EVEN) (TAG_Q_EVEN) (TAG_I_ODD) (TAG_Q_ODD) (IQpairs,even_pulse) (IQpairs,odd_pulse)
          ///  (TAG_I_EVEN) (TAG_Q_EVEN) (TAG_I_ODD) (TAG_Q_ODD) (IQpairs,even_pulse) (IQpairs,odd_pulse)
          ///  (TAG_I_EVEN) (TAG_Q_EVEN) (TAG_I_ODD) (TAG_Q_ODD) (IQpairs,even_pulse) (IQpairs,odd_pulse)
          ///  (TAG_I_EVEN) (TAG_Q_EVEN) (TAG_I_ODD) (TAG_Q_ODD) (IQpairs,even_pulse) (IQpairs,odd_pulse)
          ///
          ///  bits 31:28  Format number   0-15(4 bits) (==2)
          ///  bits 27:26  Channel number  0-3 (2 bits)
          ///  bits    25  0=even, 1=odd   0-1 (1 bit)
          ///  bit     24  0=I, 1=Q        0-1 (1 bit)
          ///  bits 23:00  Sequence number     (24 bits)

          uint32_t simPulseNum = _sd3c.nextSimPulseNum(_chanId);
          for (int freq = 0; freq < 4; freq++) {
            for (size_t j = 0; j < sizeof(uint32_t); j++) {
              //uint32_t tag = ciMakeTag(2, _chanId, (j>>1)&1, j&1, _simPulseNum);
              uint32_t tag = ciMakeTag(2, _chanId, (j>>1)&1, j&1, simPulseNum);
              char* p = (char*)&tag;
              for (size_t i = 0; i < sizeof(uint32_t); i++) {
                _simFifo.push_back(p[i]);
              }
            }
          }

          // Add IQ data. Occasionally drop some data
          bool doBadSync = ((1.0 * rand())/RAND_MAX) < 5.0e-6;
          // Disable corrupted sync data for now.
          doBadSync = false;
          // I and Q values from the CI are 4 byte values,
          // so it will take 8 bytes for an I/Q pair.
          int nPairs = _beamLength/8;
          if (doBadSync) {
            nPairs = (int)(((1.0 * rand())/RAND_MAX) * nPairs);
          }
          for (int i = 0; i < nPairs; i++) {
            char iq[8];
            int nread = p716xDn::_simulatedRead(iq, 8);
            assert(nread == 8);
            for (int j = 0; j < 8; j++) {
              _simFifo.push_back(iq[j]);
            }
          }

          break;
        }
      }
    }
  }

  //////////////////////////////////////////////////////////////////////////////////
  void
    p716xDn_sd3c::unpackPtMetadata(uint32_t *metadata,
                                   uint32_t &pulseNum,
                                   uint32_t &chanNum,
                                   double &angle1,
                                   double &angle2,
                                   bool &xmitPolHorizontal) {
    // pulse number
    pulseNum = metadata[0];
    // channel number
    chanNum = (metadata[1] & 0x6) >> 1;
    // hv flag
    xmitPolHorizontal = (metadata[1] & 0x1) == 0;
    // The first 32-bit word is the rotation/azimuth angle
    angle1 = (360. / 65536.0) * metadata[2];
    // The second 32-bit word is the tilt/elevation angle
    angle2 = (360. / 65536.0) * metadata[3];
    // Move angle2 into range [-180,180]
    if (angle2 > 180.0) {
      angle2 -= 360.0;
    }
    // metadata[4] and metadata[5] are not used

  }

  //////////////////////////////////////////////////////////////////////////////////
  void
    p716xDn_sd3c::dumpSimFifo(std::string label, int n) {
    //boost::recursive_mutex::scoped_lock guard(_mutex);
    std::ostringstream out;
    out << label <<  " _simFifo length: " << _simFifo.size() << std::endl;
    out << std::hex;
    for (unsigned int i = 0; i < (unsigned int)n && i < _simFifo.size(); i++) {
      out << std::hex << std::setw(2) << std::setfill('0') << (int)(unsigned char)_simFifo[i] << " ";
      if (!((i+1) % 40)) {
        out << std::endl;
      }
    }
    out << std::dec;
    DLOG << out.str();
  }

} // end namespace Pentek
