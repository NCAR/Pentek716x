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
    DDCDECIMATETYPE simulateDDCType) : 
        p7142(devName, simulate),
        _staggeredPrt(staggeredPrt),
        _freeRun(freeRun),
        _gates(gates),
        _nsum(nsum),
        _simulateDDCType(simulateDDCType) {
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

    // sync pulse timer
    _setTimer(MASTER_SYNC_TIMER, 0, 4);
    
    // tx pulse pulse timer
    int txDelayCounts = timeToCounts(tx_delay);
    int pulseWidthCounts = timeToCounts(tx_pulsewidth);
    _setTimer(TX_PULSE_TIMER, txDelayCounts, pulseWidthCounts);
    
//    std::cout << "downconverter: " << ddcTypeName(_ddcType) << std::endl;
//    std::cout << "rx 0/1 delay:  " << _timerDelay(RX_01_TIMER) << " adc_clock/2 counts"  << std::endl; 
//    std::cout << "rx 0/1 width:  " << _timerWidth(RX_01_TIMER) << " adc_clock/2 counts"   << std::endl;
//    std::cout << "rx 2/3 delay:  " << _timerDelay(RX_23_TIMER) << " adc_clock/2 counts"  << std::endl; 
//    std::cout << "rx 2/3 width:  " << _timerWidth(RX_23_TIMER) << " adc_clock/2 counts"   << std::endl;
//    std::cout << "tx delay:      " << _timerDelay(TX_PULSE_TIMER) << " adc_clock/2 counts"  << std::endl;
//    std::cout << "tx pulse width:" << _timerWidth(TX_PULSE_TIMER) << " adc_clock/2 counts"   << std::endl;
//    std::cout << "gate spacing:  " << gateSpacing()    << " m"                    << std::endl;
//    std::cout << "prt:           " << _prtCounts       << " adc_clock/2 counts"   << std::endl;
//    std::cout << "prt2:          " << _prt2Counts      << " adc_clock/2 counts"   << std::endl;
//    std::cout << "staggered:     " << ((_staggeredPrt) ? "true" : "false")        << std::endl;
//    std::cout << "rng to gate0:  " << rangeToFirstGate() << " m"                  << std::endl;
//    std::cout << "clock source:  " << (usingInternalClock() ? "internal" : "external") << std::endl;
//    std::cout << "ts length:     " << _tsLength                                   << std::endl;
//    std::cout << "gates:         " << _gates                                      << std::endl;
//    std::cout << "nsum:          " << _nsum                                       << std::endl;
//    std::cout << "free run:      " << ((_freeRun) ? "true" : "false")             << std::endl;
//    std::cout << "adc clock:     " << _adc_clock       << " Hz"                   << std::endl;
//    std::cout << "prf:           " << _prf             << " Hz"                   << std::endl;
//    std::cout << "data rate:     " << dataRate()/1.0e3 << " KB/s"                 << std::endl;
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
p7142sd3c::_setTimer(TimerIndex ndx, int delay, int width, bool verbose) {
    _DelayAndWidth currentVals = _timers[ndx];
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
    _timers[ndx] = _DelayAndWidth(delay, width);
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

    if (_simulate)
        return;
        
    // Load timer values before starting the timers
    if (start) {
        _initTimers();
    }
        
    // Turn on Write Strobes
    _pp.offset = MT_WR;
    _pp.value = WRITE_ON;
    ioctl(ctrlFd(), FIOREGSET, &_pp);

    // Control Register
    _pp.offset = MT_ADDR;
    _pp.value = CONTROL_REG | ALL_SD3C_TIMER_BITS;
    ioctl(ctrlFd(), FIOREGSET, &_pp);

    // Enable/Disable Timer
    _pp.offset = MT_DATA;
    if (start) {
        _pp.value = TIMER_ON;
    } else {
        _pp.value = 0;
    }
    ioctl(ctrlFd(), FIOREGSET, &_pp);

    // Force internal triggers for now.
    bool internalTriggers = true;

    _pp.offset = MT_ADDR; // Address
    if (start) {
        if (internalTriggers)
            _pp.value = ALL_SD3C_TIMER_BITS | ADDR_TRIG;  // internal trigger
        else
            _pp.value = ALL_SD3C_TIMER_BITS | GPS_EN;     // external trigger
    } else {
        _pp.value = ALL_SD3C_TIMER_BITS;
    }
    ioctl(ctrlFd(), FIOREGSET, &_pp);

    if (start) {
        _pp.value = ALL_SD3C_TIMER_BITS;
        ioctl(ctrlFd(), FIOREGSET, &_pp);
    }

    // Turn off Write Strobes
    _pp.offset = MT_WR;
    _pp.value = WRITE_OFF;
    ioctl(ctrlFd(), FIOREGSET, &_pp);

    if (start) {
        std::cout << "timers started" << std::endl;
    } else {
        std::cout << "timers stopped" << std::endl;
        //exit (1);
    }

    // Get current system time as xmit start time
    setXmitStartTime(boost::posix_time::microsec_clock::universal_time());
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
    _pp.offset = KAISER_ADDR;
    ioctl(ctrlFd(), FIOREGGET, &_pp);
    _pp.offset = KAISER_ADDR;
    _pp.value = DDC_STOP;
    ioctl(ctrlFd(), FIOREGSET, &_pp);
    usleep(p7142::P7142_IOCTLSLEEPUS);
    _pp.offset = KAISER_ADDR;
    ioctl(ctrlFd(), FIOREGGET, &_pp);
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

////////////////////////////////////////////////////////////////////////////////////////
void
p7142sd3c::_resetDCM() {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (isSimulating())
        return;

    _pp.offset = DCM_CONTROL;

    // read the dcm control register
    ioctl(ctrlFd(), FIOREGGET, &_pp);
    //std::cout << "DCM control readback is 0x" << std::hex << _pp.value << std::endl;

    // turn on the DCM reset bit
    _pp.value = 0x10 | _pp.value;
    ioctl(ctrlFd(), FIOREGSET, &_pp);
    usleep(1000);

    ioctl(ctrlFd(), FIOREGGET, &_pp);
    //std::cout << "DCM control readback is 0x" << std::hex << _pp.value << std::endl;

    // turn off the DCM reset bit
    _pp.value = _pp.value & ~0x10;
    ioctl(ctrlFd(), FIOREGSET, &_pp);
    usleep(1000);

    ioctl(ctrlFd(), FIOREGGET, &_pp);
    //std::cout << "DCM control readback is 0x" << std::hex << _pp.value << std::endl;

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

    std::cout << "free run mode is " << (_freeRun ? "enabled" : "disabled") << 
        " for " << devName() << std::endl;
    std::cout << "transceiver control register is " << _pp.value << std::endl;
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
            _timerWidth(i) << ")" << std::endl;
        
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

unsigned int
p7142sd3c::_controlIoctl(int request, unsigned int offset, unsigned int value) {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    _pp.offset = offset;
    _pp.value = value;
    ioctl(ctrlFd(), request, &_pp);
    usleep(p7142::P7142_IOCTLSLEEPUS);
    return _pp.value;
}

} // end namespace Pentek
