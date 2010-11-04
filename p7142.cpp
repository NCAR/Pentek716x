#include "p7142.h"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <iostream>
#include <cstdio>
#include <cstdlib>

using namespace Pentek;
int drv_peekL(int fd,unsigned intaddr);
int drv_pokeL(int fd,unsigned int addr,unsigned int value);

////////////////////////////////////////////////////////////////////////////////
p7142::p7142(std::string devName, bool simulate):
  p71xx(devName, simulate), _downconverters(P7142_NCHANNELS), _upconverter(0)
{
}

////////////////////////////////////////////////////////////////////////////////
p7142::~p7142() {
    boost::recursive_mutex::scoped_lock guard(_mutex);
    for (int i = 0; i < P7142_NCHANNELS; i++) {
        delete _downconverters[i];
    }
}

////////////////////////////////////////////////////////////////////////////////
p7142Dn *
p7142::addDownconverter(int chanId, int bypassdivrate,
        int simWavelength, bool sim4bytes) {
    boost::recursive_mutex::scoped_lock guard(_mutex);
    // Just construct a new downconverter and put it in our list.
    p7142Dn * downconverter = new p7142Dn(this, chanId, bypassdivrate, 
            simWavelength, sim4bytes);
    _addDownconverter(downconverter);
    return(downconverter);
}

////////////////////////////////////////////////////////////////////////////////
p7142Up *
p7142::addUpconverter(std::string upName, double sampleClockHz,
        double ncoFreqHz, char mode) {
    boost::recursive_mutex::scoped_lock guard(_mutex);
    // Just construct a new upconverter and put it in our list.
    p7142Up * upconverter = new p7142Up(this, upName, sampleClockHz, ncoFreqHz, mode);
    _addUpconverter(upconverter);
    return(upconverter);
}

////////////////////////////////////////////////////////////////////////////////
void
p7142::_addDownconverter(p7142Dn * downconverter) {
    boost::recursive_mutex::scoped_lock guard(_mutex);
    
    int chanId = downconverter->chanId();
    if (_downconverters[chanId]) {
        std::cerr << "Existing downconverter for channel " << chanId <<
                " is being replaced on device " << _devName << std::endl;
        delete _downconverters[chanId];
    }
    _downconverters[chanId] = downconverter;
}

////////////////////////////////////////////////////////////////////////////////
void
p7142::_addUpconverter(p7142Up * upconverter) {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (_upconverter) {
        std::cerr << "Existing upconverter is being replaced on device " << 
                _devName << std::endl;
        delete _upconverter;
    }
    _upconverter = upconverter;
}

////////////////////////////////////////////////////////////////////////////////
unsigned int
p7142::_controlIoctl(unsigned int request, unsigned int offset, 
    unsigned int value) {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    ARG_PEEKPOKE pp;
    pp.page = 2; // PCIBAR 2
    pp.mask = 0;
    pp.offset = offset;
    pp.value = value;

    ioctl(ctrlFd(), request, &pp);
    usleep(p7142::P7142_IOCTLSLEEPUS);

    return pp.value;
}

////////////////////////////////////////////////////////////////////////////////////////
void
p7142::_resetDCM() {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (isSimulating())
        return;

    // read the DCM control register
    unsigned int val = _controlIoctl(FIOREGGET, DCM_CONTROL);

    // turn on the DCM reset bit
    val |= 0x10;
    _controlIoctl(FIOREGSET, DCM_CONTROL, val);
    usleep(1000);

    val = _controlIoctl(FIOREGGET, DCM_CONTROL);

    // turn off the DCM reset bit
    val &= ~0x10;
    _controlIoctl(FIOREGSET, DCM_CONTROL, val);
    usleep(1000);

    val = _controlIoctl(FIOREGGET, DCM_CONTROL);
}
