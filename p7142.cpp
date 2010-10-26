#include "p7142.h"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <iostream>
#include <cstdio>
#include <cstdlib>

#include <boost/pool/detail/guard.hpp>
using namespace boost::details::pool;   // for guard

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
    guard<boost::recursive_mutex> guard(_mutex);
    for (int i = 0; i < P7142_NCHANNELS; i++) {
        delete _downconverters[i];
    }
}

////////////////////////////////////////////////////////////////////////////////
p7142Dn *
p7142::addDownconverter(int chanId, int bypassdivrate,
        int simWavelength, bool sim4bytes) {
    guard<boost::recursive_mutex> guard(_mutex);
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
    guard<boost::recursive_mutex> guard(_mutex);
    // Just construct a new upconverter and put it in our list.
    p7142Up * upconverter = new p7142Up(this, upName, sampleClockHz, ncoFreqHz, mode);
    _addUpconverter(upconverter);
    return(upconverter);
}

////////////////////////////////////////////////////////////////////////////////
void
p7142::_addDownconverter(p7142Dn * downconverter) {
    guard<boost::recursive_mutex> guard(_mutex);
    
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
    guard<boost::recursive_mutex> guard(_mutex);

    if (_upconverter) {
        std::cerr << "Existing upconverter is being replaced on device " << 
                _devName << std::endl;
        delete _upconverter;
    }
    _upconverter = upconverter;
}
