/*
 * p7142sd3c.cpp
 *
 *  Created on: Jan 26, 2009
 *      Author: martinc
 */

#include "p7142sd3c.h"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <iostream>

namespace Pentek {
    
using namespace boost::posix_time;

/*
 * Timer identifier bits for the SD3C timers.
 */
const unsigned int p7142sd3c::SD3C_TIMER_BITS[N_SD3C_TIMERS] = {
        0x010, 0x020, 0x040, 0x080,
        0x100, 0x200, 0x400, 0x800
};
const unsigned int p7142sd3c::ALL_SD3C_TIMER_BITS = 0xff0;


////////////////////////////////////////////////////////////////////////////////////////
p7142sd3c::p7142sd3c(std::string devName, bool simulate, double tx_delay, 
    double tx_pulsewidth, double prt, double prt2, bool staggeredPrt, 
    unsigned int gates, unsigned int nsum, bool freeRun, 
    DDCDECIMATETYPE simulateDDCType, bool externalStartTrigger) : 
        p7142(devName, simulate),
        _staggeredPrt(staggeredPrt),
        _freeRun(freeRun),
        _gates(gates),
        _nsum(nsum),
        _simulateDDCType(simulateDDCType),
        _externalStartTrigger(externalStartTrigger) {
    boost::recursive_mutex::scoped_lock guard(_mutex);
            
    // Set up page and mask registers for FIOREGSET and FIOREGGET functions 
    // to access FPGA registers. We use _pp for all of our ioctl() calls, so
    // set it up before any ioctl-s!
    _pp.page = 2; // PCIBAR 2
    _pp.mask = 0;
    
    // Note the FPGA firmware revision
    _fpgaRepoRev = _readFpgaRepoRevision();
    std::cout << _devName << " FPGA revision: " << _fpgaRepoRev << std::endl;
    if (_fpgaRepoRev == 0) {
        std::cerr << "** WARNING: Revision number is zero. " <<
                "Was the correct firmware loaded?" << std::endl;
    }

    // sanity check
    if (_nsum < 1) {
        std::cerr << "Selected nsum of " << _nsum << " makes no sense!" <<
                std::endl;
        abort();
    }
    
    // Determine our operating mode
    _mode = (_nsum > 1) ? MODE_CI : MODE_PULSETAG;
    if (_freeRun)
        _mode = MODE_FREERUN;

    // Set the ADC clock rate based on DDC type
    _ddcType = _readDDCType();
    std::cout << _devName << " DDC type: " << ddcTypeName() << std::endl;
    switch (_ddcType) {
    case DDC10DECIMATE:
        _adc_clock = 100.0e6;
        break;
    case DDC8DECIMATE:
        _adc_clock = 125.0e6;
        break;
    case DDC4DECIMATE:
        _adc_clock = 48.0e6;
        break;
    case BURST:
        _adc_clock = 100.0e6;
        break;    
    }
    
    // stop the timers
    timersStartStop(false);
    
    // Write the gate count and coherent integration registers
    if (! isSimulating()) {
        _controlIoctl(FIOREGSET, RADAR_GATES, gates);
        _controlIoctl(FIOREGGET, RADAR_GATES);

        _controlIoctl(FIOREGSET, CI_NSUM, nsum);
        _controlIoctl(FIOREGGET, CI_NSUM);
    }

    // Convert prt, prt2, tx_pulsewidth, and tx_delay into our local representation, 
    // which is in units of (ADC clock counts / 2)
    _prtCounts = timeToCounts(prt);
    _prt2Counts = timeToCounts(prt2);
    _prf = 1.0 / prt;   // Hz
    _prf2 = 1.0 / prt2; // Hz

    // Sync pulse timer. Note that the width of this timer must be at least
    // 140 ns to be recognized to be counted by the Acromag PMC730 Multi-IO
    // card pulse counter, and this counter is used by the Ka-band radar!
    _setTimer(MASTER_SYNC_TIMER, 0, timeToCounts(140.e-9));
    
    // tx pulse timer
    int txDelayCounts = timeToCounts(tx_delay);
    int pulseWidthCounts = timeToCounts(tx_pulsewidth);
    _setTimer(TX_PULSE_TIMER, txDelayCounts, pulseWidthCounts);
    
    std::cout << "downconverter: " << ddcTypeName(_ddcType) << std::endl;
//    std::cout << "rx 0/1 delay:  " << _timerDelay(RX_01_TIMER) << " adc_clock/2 counts"  << std::endl; 
//    std::cout << "rx 0/1 width:  " << _timerWidth(RX_01_TIMER) << " adc_clock/2 counts"   << std::endl;
//    std::cout << "rx 2/3 delay:  " << _timerDelay(RX_23_TIMER) << " adc_clock/2 counts"  << std::endl; 
//    std::cout << "rx 2/3 width:  " << _timerWidth(RX_23_TIMER) << " adc_clock/2 counts"   << std::endl;
    std::cout << "tx delay:      " << _timerDelay(TX_PULSE_TIMER) << " adc_clock/2 counts"  << std::endl;
    std::cout << "tx pulse width:" << _timerWidth(TX_PULSE_TIMER) << " adc_clock/2 counts"   << std::endl;
//    std::cout << "gate spacing:  " << gateSpacing()    << " m"                    << std::endl;
    std::cout << "prt:           " << _prtCounts       << " adc_clock/2 counts"   << std::endl;
    std::cout << "prt2:          " << _prt2Counts      << " adc_clock/2 counts"   << std::endl;
    std::cout << "staggered:     " << ((_staggeredPrt) ? "true" : "false")        << std::endl;
//    std::cout << "rng to gate0:  " << rangeToFirstGate() << " m"                  << std::endl;
//    std::cout << "clock source:  " << (usingInternalClock() ? "internal" : "external") << std::endl;
//    std::cout << "ts length:     " << _tsLength                                   << std::endl;
    std::cout << "gates:         " << _gates                                      << std::endl;
    std::cout << "nsum:          " << _nsum                                       << std::endl;
    std::cout << "free run:      " << ((_freeRun) ? "true" : "false")             << std::endl;
    std::cout << "adc clock:     " << _adc_clock       << " Hz"                   << std::endl;
    std::cout << "prf:           " << _prf             << " Hz"                   << std::endl;
    std::cout << "data rate:     " << dataRate()/1.0e3 << " KB/s"                 << std::endl;
//    std::cout << "sim usleep     " << _simPauseMS*1000 << "us"                    <<std::endl;
//    for (int i = 0; i < 8; i++) {
//        std::cout << "timer " << i << " delay: " << _timerDelay(i) << " adc_clock/2 counts"  << std::endl;
//        std::cout << "timer " << i << " width: " << _timerWidth(i) << " adc_clock/2 counts"  << std::endl;
//    }
    
    // reset the FPGA clock managers. Necessary since some of our
    // new DCMs in the firmware use the CLKFX output, which won't
    // lock at startup.
    _resetDCM();

    // set free run mode as appropriate
    _loadFreeRun();
}

////////////////////////////////////////////////////////////////////////////////////////
p7142sd3c::~p7142sd3c() {
}

////////////////////////////////////////////////////////////////////////////////////////
p7142sd3cDn *
p7142sd3c::addDownconverter(int chanId, bool burstSampling, int tsLength,
        double rx_delay, double rx_pulse_width, std::string gaussianFile, 
        std::string kaiserFile, double simPauseMs, int simWavelength,
        bool internalClock) {
    boost::recursive_mutex::scoped_lock guard(_mutex);
    // Create a new p7142sd3cDn downconverter and put it in our list
    p7142sd3cDn * downconverter = new p7142sd3cDn(this, chanId, burstSampling,
            tsLength, rx_delay, rx_pulse_width, gaussianFile, kaiserFile, 
            simPauseMs, simWavelength, internalClock);
    _addDownconverter(downconverter);
    return(downconverter);
}

////////////////////////////////////////////////////////////////////////////////////////
void
p7142sd3c::_setTimer(TimerIndex ndx, int delay, int width, bool verbose, bool invert) {
    _TimerConfig currentVals = _timerConfigs[ndx];
    boost::recursive_mutex::scoped_lock guard(_mutex);
    // If current timer width is non-zero, warn about any changes in 
    // width or delay.
    if (verbose && currentVals.width() != 0) {
        if (currentVals.width() != width) {
            std::cerr << "WARNING: Width for timer " << ndx << 
                    " is changing from " << currentVals.width() << " to " <<
                    width << std::endl;
        }
        if (currentVals.delay() != delay) {
            std::cerr << "WARNING: Delay for timer " << ndx << 
                    " is changing from " << currentVals.delay() << " to " <<
                    delay << std::endl;
        }
    }
    _timerConfigs[ndx] = _TimerConfig(delay, width, invert);
}

//////////////////////////////////////////////////////////////////////////////////
int
p7142sd3c::timeToCounts(double time) const {
    boost::recursive_mutex::scoped_lock guard(_mutex);
    return(lround(time * _adc_clock / 2));
}

//////////////////////////////////////////////////////////////////////////////////
double
p7142sd3c::countsToTime(int counts) const {
    boost::recursive_mutex::scoped_lock guard(_mutex);
    return((2 * counts) / _adc_clock);
}

//////////////////////////////////////////////////////////////////////
unsigned int p7142sd3c::_readFpgaRepoRevision() {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (_simulate)
        return 1;
    
    _pp.offset = FPGA_REPO_REV;
    ioctl(ctrlFd(), FIOREGGET, &_pp);
    return _pp.value & 0x3fff;

}

/////////////////////////////////////////////////////////////////////////
void p7142sd3c::timersStartStop(bool start) {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (_simulate) {
        setXmitStartTime(microsec_clock::universal_time());
        return;
    }

        
    // Load timer values before starting the timers
    if (start) {
        _initTimers();
    }
        
    // Turn on Write Strobes
    _controlIoctl(FIOREGSET, MT_WR, WRITE_ON);

    // configure each timer
    for (int i = 0; i < 8; i++) {
	    // Control Register
        _controlIoctl(FIOREGSET, MT_ADDR, CONTROL_REG | SD3C_TIMER_BITS[i]);
	
	    // Enable/Disable Timer
        unsigned int value = (start ? TIMER_ON : 0) | 
                (_timerInvert(i) ? TIMER_NEG : 0);
        _controlIoctl(FIOREGSET, MT_DATA, value);
    }
    // Get current time
    ptime now(microsec_clock::universal_time());
    //
    // Actually start or stop the timers now
    //
    if (start) {
        if (_externalStartTrigger) {
            // We assume here that the external trigger is a 1 PPS signal, 
            // e.g., from GPS.
            //
            // Sleep until ~0.2 seconds after the top of a second. This gives
            // us a comfortable fraction of a second to set up timer start and 
            // know at precisely which second the timers will start. It also 
            // allows for our system clock to be off by up to 0.2 seconds.
            
            // sleep until the next 0.2 second mark
            int wake_uSec = 200000; // wake at 0.2 seconds after the top of a second
            int usecNow = now.time_of_day().total_microseconds() % 1000000;
            int sleep_uSec = (1000000 + wake_uSec - usecNow) % 1000000;
            // Timers will start at the top of the next second after we wake
            setXmitStartTime(now + microseconds(1000000 + sleep_uSec - wake_uSec));
            // Now sleep
            usleep(sleep_uSec);
            // Set the wait-for-trigger bit so timers start at the next
            // trigger.
            _controlIoctl(FIOREGSET, MT_ADDR, ALL_SD3C_TIMER_BITS | GPS_EN);
        } else {
            // Internal trigger: timers start immediately.
            setXmitStartTime(now);
            _controlIoctl(FIOREGSET, MT_ADDR, ALL_SD3C_TIMER_BITS | ADDR_TRIG);
        }
        
        std::cout << "Timers/radar start time " << _xmitStartTime << std::endl;
    } else {
        _controlIoctl(FIOREGSET, MT_ADDR, ALL_SD3C_TIMER_BITS);
        std::cout << "Timers stopped at " << now << std::endl;
    }
    
    // Turn off Write Strobes
    _controlIoctl(FIOREGSET, MT_WR, WRITE_OFF);
}

//////////////////////////////////////////////////////////////////////
void p7142sd3c::startFilters() {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (isSimulating())
        return;

    // Start the DDC
    _pp.offset = KAISER_ADDR;
    _pp.value = DDC_START;
    ioctl(ctrlFd(), FIOREGSET, &_pp);
    usleep(p7142::P7142_IOCTLSLEEPUS);

    std::cout << "filters enabled on " << _devName << std::endl;
}

//////////////////////////////////////////////////////////////////////
void p7142sd3c::stopFilters() {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (isSimulating())
        return;

    // stop the filters if they are running.
    _controlIoctl(FIOREGGET, KAISER_ADDR);  
    _controlIoctl(FIOREGSET, KAISER_ADDR, DDC_STOP);
    _controlIoctl(FIOREGGET, KAISER_ADDR);
}

//////////////////////////////////////////////////////////////////////
unsigned short int p7142sd3c::TTLIn() {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (_simulate)
        return 0;

    _pp.offset = TTL_IN;
    ioctl(ctrlFd(), FIOREGGET, &_pp);

    return _pp.value;
}

//////////////////////////////////////////////////////////////////////
void p7142sd3c::TTLOut(unsigned short int data) {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (_simulate)
        return;

    _pp.value = data;

    _pp.offset = TTL_OUT1;
    ioctl(ctrlFd(), FIOREGSET, &_pp);

}

//////////////////////////////////////////////////////////////////////
p7142sd3c::DDCDECIMATETYPE p7142sd3c::_readDDCType() {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (_simulate)
        return _simulateDDCType;
    
    _pp.offset = FPGA_REPO_REV;
    ioctl(ctrlFd(), FIOREGGET, &_pp);
    
    // Up to rev 502, DDC type was a 1-bit value at bit 15.
    // After that it's a 2-bit value in bits 14-15.
    int ddcTypeFpgaVal = (_fpgaRepoRev > 502) ? 
        (_pp.value & 0xC000) >> 14 : (_pp.value & 0x8000) >> 15;
    
    DDCDECIMATETYPE ddctype = DDC4DECIMATE;
    switch (ddcTypeFpgaVal) {
    case 0:
        ddctype = DDC4DECIMATE;
        break;
    case 1:
        ddctype = DDC8DECIMATE;
        break;
    case 2:
        ddctype = DDC10DECIMATE;
        break;
    case 3:
        ddctype = BURST;
        break;     
    }
    
    return ddctype;

}

//////////////////////////////////////////////////////////////////////
void
p7142sd3c::_loadFreeRun() {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (isSimulating())
        return;

    // set the free run bit as needed

    // get the current state of transceiver control register
    _pp.offset = TRANS_CNTRL;
    ioctl(ctrlFd(), FIOREGGET, &_pp);
    usleep(P7142_IOCTLSLEEPUS);

    if (_freeRun) {
        // set free run
        _pp.value = _pp.value | TRANS_FREE_RUN;
    } else {
        // clear free run
        _pp.value = _pp.value & ~TRANS_FREE_RUN;
    }

    // write transceiver control register
    ioctl(ctrlFd(), FIOREGSET, &_pp);
    usleep(P7142_IOCTLSLEEPUS);

    ioctl(ctrlFd(), FIOREGGET, &_pp);
    usleep(P7142_IOCTLSLEEPUS);

    //std::cout << "free run mode is " << (_freeRun ? "enabled" : "disabled") << 
    //    " for " << devName() << std::endl;
    //std::cout << "transceiver control register is " << _pp.value << std::endl;
}

/////////////////////////////////////////////////////////////////////////
bool
p7142sd3c::_initTimers() {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (_simulate)
        return true;

    //
    //    This section initializes the timers.

    int periodCount; // Period Count for all Timers
    int PrtScheme; // PRT Scheme for all Timers

    // Calculate the period and PRT Scheme for dual prt or single prt
    // Note: _prtCounts and _prt2Counts are expressed in ADC_Clk/2 MHz Counts!
    //       for DDC4: 24 MHz; for DDC8: 62.5 MHz

    int X, Y;
    float prt_ms, prt2_ms;

    if (_staggeredPrt == true) {
        // dual prt
        prt_ms = countsToTime(_prtCounts) * 1000;
        prt2_ms = countsToTime(_prt2Counts) * 1000;

        periodCount = timeToCounts(prt_ms * (prt2_ms / prt_ms - (int) (prt2_ms
                / prt_ms)) / (int) (prt2_ms / prt_ms) * 0.001);

        X = (int) ((int) (prt2_ms / prt_ms) / 
                (prt2_ms / prt_ms - (int) (prt2_ms / prt_ms)));
        Y = (int) (X * prt2_ms / prt_ms);

        PrtScheme = (Y << 4) | X;
    } else {
        //single prt
        //  PRT must be integral multiple of pulsewidth !
        periodCount = _prtCounts;
        PrtScheme = 0x0000;
    }

    std::cout << "periodCount is " << periodCount << std::endl;

    // Control Register
    _pp.offset = MT_ADDR;
    _pp.value = CONTROL_REG | ALL_SD3C_TIMER_BITS;
    ioctl(ctrlFd(), FIOREGSET, &_pp);

    // Enable Timer
    _pp.offset = MT_DATA;
    _pp.value = TIMER_ON;
    ioctl(ctrlFd(), FIOREGSET, &_pp);

    // Turn on Write Strobes
    _pp.offset = MT_WR;
    _pp.value = WRITE_ON;
    ioctl(ctrlFd(), FIOREGSET, &_pp);
    
    for (unsigned int i = 0; i < N_SD3C_TIMERS; i++) {
        std::cout << "Initializing timer " << i << ": delay " <<
            countsToTime(_timerDelay(i)) << "s (" << _timerDelay(i) <<
            "), width " << countsToTime(_timerWidth(i)) << "s (" << 
            _timerWidth(i) << ")" << (_timerInvert(i)? ", inverted":"") << std::endl;
        
        // Delay Register
        _pp.offset = MT_ADDR; // Address
        _pp.value = DELAY_REG | SD3C_TIMER_BITS[i];
        ioctl(ctrlFd(), FIOREGSET, &_pp);

        _pp.offset = MT_DATA; // Data
        _pp.value = _timerDelay(i);
        ioctl(ctrlFd(), FIOREGSET, &_pp);

        // Pulse Width Register
        _pp.offset = MT_ADDR; // Address
        _pp.value = WIDTH_REG | SD3C_TIMER_BITS[i];
        ioctl(ctrlFd(), FIOREGSET, &_pp);

        _pp.offset = MT_DATA; // Data
        _pp.value = _timerWidth(i);
        ioctl(ctrlFd(), FIOREGSET, &_pp);
    }

    // ALL TIMERS
    // Period Register
    _pp.offset = MT_ADDR; // Address
    _pp.value = PERIOD_REG | ALL_SD3C_TIMER_BITS;
    ioctl(ctrlFd(), FIOREGSET, &_pp);

    _pp.offset = MT_DATA; // Data
    _pp.value = periodCount;
    ioctl(ctrlFd(), FIOREGSET, &_pp);

    //Multiple PRT Register
    _pp.offset = MT_ADDR; // Address
    _pp.value = PRT_REG | ALL_SD3C_TIMER_BITS;
    ioctl(ctrlFd(), FIOREGSET, &_pp);

    _pp.offset = MT_DATA; // Mult PRT Valu Timer 0
    _pp.value = PrtScheme;
    ioctl(ctrlFd(), FIOREGSET, &_pp);

    // Turn off Write Strobes
    _pp.offset = MT_WR;
    _pp.value = WRITE_OFF;
    ioctl(ctrlFd(), FIOREGSET, &_pp);

    return true;

}

//////////////////////////////////////////////////////////////////////
int p7142sd3c::dataRate() {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    int rate = 0;

    switch (_mode) {
    case MODE_FREERUN:
        // two bytes of I and two bytes of Q for each range gate
        rate = _gates*4;
        break;
    case MODE_PULSETAG:
        // pulse tagger
        // there is a four byte sync word and a four byte pulse tag
        // at the beginning of each pulse. There are two bytes for each
        // I and each Q for each range gate.
        rate = (int)(_prf * (4 + 4 + _gates*4));
        break;
    case MODE_CI:
        // coherent integration
        // there is a 16 byte tag at the beginning of each pulse. Each pulse
        // returns a set of even I's and Q's, and a set of odd I's and Q's. The
        // even and odd pulses are separated by the prt, and so taken together they
        // run at half the prf. Each I and Q for a gate is 32 bits (wider than the
        // non-CI mode because they are sums of 16 bit numbers), so there are 8 bytes
        // per gate for even and 8 bytes per gate for odd pulses.
        rate = (int)((_prf/2)*(16+_gates*8*2)/_nsum);
        break;
    }

    return rate;
}

} // end namespace Pentek
