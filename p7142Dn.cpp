#include "p7142Dn.h"

#include <fcntl.h>
#include <iostream>
#include <cassert>
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sys/ioctl.h>

#include <logx/Logging.h>
LOGGING("p7142Dn");

using namespace Pentek;

////////////////////////////////////////////////////////////////////////////////
p7142Dn::p7142Dn(
        p7142* p7142,
        int chanId,
        uint32_t dmaDescSize,
        int bypassdivrate,
        int simWaveLength,
        bool sim4bytes,
        bool internalClock) :
  _p7142(*p7142),
  _chanId(chanId),
  _DmaDescSize(dmaDescSize),
  _bytesRead(0),
  _simWaveLength(simWaveLength),
  _angleCount(0),
  _sim4bytes(sim4bytes),
  _mutex(),
  _adcActive(false),
  _readBufAvail(0),
  _readBufOut(0)
{
    // dma buffer size must be a multiple of 4
    if ((_DmaDescSize % 4) || (_DmaDescSize <= 0)) {
      ELOG << "DMA descriptor size must be a positive  multiple of 4 bytes, "
           <<  _DmaDescSize << " was specified";
      abort();
    }

    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (isSimulating()) {
        _adcActive = true;
        return;
    }

    // Put a set of _DmaDescSize char buffers in the free buffer queue
    // for this channel. These buffers are used when pulling data 
    // from DMA.
    for (int i = 0; i < N_READYFLOW_DMA_BUFFERS; i++) {
        _freeBuffers.push(new char[_DmaDescSize]);
    }
    // Size _readBuf appropriately.
    _readBuf.resize(2 * _DmaDescSize);

    // Set up the fifo and DMA
    _initFifoAndDma();
    
    // Start DMA
    _start();

    // Set the clock source. This is pretty bogus, because currently the last downconverter
    // instantiated sets the clock source for all of them...
    uint32_t clockSource = internalClock ?
            P7142_MSTR_CTRL_SEL_CLK_OSCILLATOR : P7142_MSTR_CTRL_SEL_CLK_EXT_CLK;
    P7142_SET_MSTR_BUS_CTRL_SEL_CLK(_p7142._p7142Regs.BAR2RegAddr.masterAControl, clockSource);
}

////////////////////////////////////////////////////////////////////////////////
p7142Dn::~p7142Dn() {
    boost::recursive_mutex::scoped_lock guard(_mutex);
    // Stop DMA and free DMA resources
    _stop();
    // Delete the buffers we allocated
    while (! _freeBuffers.empty()) {
        char * buf = _freeBuffers.front();
        _freeBuffers.pop();
        delete(buf);
    }
    while (! _filledBuffers.empty()) {
        char * buf = _filledBuffers.front();
        _filledBuffers.pop();
        delete(buf);
    }
}

////////////////////////////////////////////////////////////////////////////////
int
p7142Dn::read(char* buf, int bufsize) {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    // Enforce that reads are a multiple of 4 bytes, since the Pentek driver
    // (silently) does this, e.g., it will return 4 bytes if 7 are requested,
    // or 0 bytes if 1 is requested.
    // If we are to support other read sizes, we'll have to be a lot smarter, 
    // and keep around a buffer of up to 3 unconsumed bytes between calls here. 
    // or now, we are not that smart...
    if ((bufsize % 4) != 0) {
      ELOG << __PRETTY_FUNCTION__ << ": " << bufsize << 
        " bytes requested, but Pentek reads must be a multiple of 4 bytes!";
      abort();
    }

    int n = isSimulating() ? _simulatedRead(buf, bufsize) : _read(buf, bufsize);

    if (n > 0)
        _bytesRead += n;

    return n;
}

////////////////////////////////////////////////////////////////////////////////////////
int
p7142Dn::_simulatedRead(char* buf, int bufsize) {
    // In simulation mode, create some random values
    // and return a full buffer.

    // there is a bug in this code. It assumes that bufsize
    // is an integral number of I/Q pairs. For the time being,
    // detect if this is not true and just abort.
    /// @todo Fix read function to return an arbitrary number of bytes
    /// when in simulation mode
    assert ((bufsize % 4) == 0);

    short* sbuf = (int16_t*)buf;
    int*   ibuf = (int32_t*)buf;

    // 4  or 8 bytes per IQ pair
    int bytesPerPair = _sim4bytes ? 8 : 4;
    int nPairs = bufsize / bytesPerPair;

    for (int p = 0; p < nPairs; p++) {
        // Noisy sine wave
        if (_angleCount == _simWaveLength) {
            _angleCount = 0;
        }
        double angle = ((double)_angleCount++)/ _simWaveLength;
        double iNoise = 5.0 * ((2.0 * rand()) / RAND_MAX - 1.0);
        double qNoise = 5.0 * ((2.0 * rand()) / RAND_MAX - 1.0);
        double I = 10000 * (sin((2 * angle * M_PI)) + iNoise);
        double Q = 10000 * (cos((2 * angle * M_PI)) + qNoise);
        if (_sim4bytes) {
            *ibuf++ = (int32_t) I; // I
            *ibuf++ = (int32_t) Q; // Q
        } else {
            *sbuf++ = (int16_t) I; // I
            *sbuf++ = (int16_t) Q; // Q
        }
    }

    return bufsize;
}

////////////////////////////////////////////////////////////////////////////////
long
p7142Dn::bytesRead() {
    //boost::recursive_mutex::scoped_lock guard(_mutex);
    long retval = _bytesRead;
    _bytesRead = 0;
    return retval;
}

////////////////////////////////////////////////////////////////////////////////
bool
p7142Dn::usingInternalClock() const {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (isSimulating())
        return(false);
    
    uint32_t clkSel;

    P7142_GET_MSTR_BUS_CTRL_SEL_CLK(_p7142._p7142Regs.BAR2RegAddr.masterAControl, clkSel);
    return (clkSel != P7142_MSTR_CTRL_SEL_CLK_EXT_CLK);

}

////////////////////////////////////////////////////////////////////////////////
int
p7142Dn::bypassDivider() const {

	if (isSimulating())
        return(0);
    
    return (*_p7142._p7142Regs.BAR2RegAddr.adcFifo[_chanId].FifoDecimationDivide) + 1;
}

////////////////////////////////////////////////////////////////////////////////
bool
p7142Dn::setBypassDivider(int bypassdiv) const {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    if (isSimulating())
        return true;
    
    unsigned int newDecimDiv = bypassdiv - 1;
    *_p7142._p7142Regs.BAR2RegAddr.adcFifo[_chanId].FifoDecimationDivide = newDecimDiv;

    if (*_p7142._p7142Regs.BAR2RegAddr.adcFifo[_chanId].FifoDecimationDivide != newDecimDiv) {
    	return false;
    }

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////
void
p7142Dn::_initFifoAndDma() {

    ////////////////////////// Fifo setup ///////////////////////////////

    // Disable the fifo.
	P7142_SET_FIFO_CTRL_FIFO_ENABLE(_p7142._p7142Regs.BAR2RegAddr.adcFifo[_chanId].FifoCtrl,
			P7142_FIFO_DISABLE);

	// Set the fifo for this ADC channel to the parameters were set
	// by p7142::_configInParameters()
	P7142InitFifoRegs(&_p7142._p7142InParams.adcFifo[_chanId],
			&(_p7142._p7142Regs.BAR2RegAddr.adcFifo[_chanId]), P7142_FIFO_TYPE_ADC);

    // Flush the fifo. The fifo will be reset. The almost empty and the almost full
	// thresholds will also be set, and the fifo will be disabled.
	P7142FlushFifo(&(_p7142._p7142Regs.BAR2RegAddr.adcFifo[_chanId]),
			&(_p7142._p7142InParams.adcFifo[_chanId]));

    // Enable the fifo. The fen signal will not be active until the
	// global gate is enabled.
	P7142_SET_FIFO_CTRL_FIFO_ENABLE(_p7142._p7142Regs.BAR2RegAddr.adcFifo[_chanId].FifoCtrl,
			P7142_FIFO_ENABLE);

    ////////////////////////// DMA setup ///////////////////////////////

	// Open a DMA channel (required before we allocate the buffer)
    int status = PTK714X_DMAOpen(_p7142._deviceHandle, _chanId, &_dmaHandle);
    if (status != PTK714X_STATUS_OK) {
        ELOG << __PRETTY_FUNCTION__ << ": Unable to open DMA channel " << 
          _chanId;
        abort();
    }

    // Allocate DMA buffers, one per channel/descriptor pair.
    for (int d = 0; d < 4; d++) {
        status = PTK714X_DMAAllocMem(_dmaHandle, _DmaDescSize, 
                &_dmaBuf[d], (BOOL)0);
        if (status != PTK714X_STATUS_OK) {
            ELOG << __PRETTY_FUNCTION__ << 
            ": Unable to allocate a DMA buffer for channel " << _chanId << 
            "/descriptor " << d;
            abort();
        }
    }

    /* Abort any existing transfers */
    P7142DmaAbort(&(_p7142._p7142Regs.BAR0RegAddr), _chanId);

    /* Flush DMA channel buffer */
    P7142DmaFlush(&(_p7142._p7142Regs.BAR0RegAddr), _chanId);

    // set up channel parameters */
    P7142DmaChanSetup(&(_p7142._p7142DmaParams.dmaChan[_chanId]),
            PCI7142_DMA_CMD_STAT_DMA_ENABLE,
            PCI7142_DMA_CMD_STAT_DEMAND_MODE_ENABLE,
            PCI7142_DMA_CMD_STAT_DATA_WIDTH_64,
            512,              /* Max Burst Count */
            0);              /* Transfer Interval Count */

    // configure four chained descriptors for each channel.
    /// @todo There is a cryptic note in Section 5.19 of the Pentek 7142
    /// operating manual which says the following about using the chain mode:
    /// <br>
    /// If you setup a DMA channel for a continuous data transfer (Chain
    /// bit D31 = 1 in all four Descriptors), you must ensure that the gating
    /// signal used for the transfer is not stopped before you stop
    /// the transfer or the channel may hang up.
    /// <br> They don't say what "hang up" means.
    for (int d = 0; d < 4; d++) {
        P7142DmaDescptrSetup(
                &(_p7142._p7142DmaParams.dmaChan[_chanId]),
                d,                                          /* descriptor number */
                _DmaDescSize,                               /* transfer count in bytes */
                PCI7142_DMA_DESCPTR_XFER_CNT_INTR_DISABLE,  /* DMA interrupt */
                PCI7142_DMA_DESCPTR_XFER_CNT_CHAIN_NEXT,    /* type of descriptor */
                (unsigned long)_dmaBuf[d].kernBuf);         /* buffer address */
    }


    // Flush the CPU caches
    for (int d = 0; d < 4; d++) {
        PTK714X_DMASyncCpu(&_dmaBuf[d]);
    }

    // Initialize the dma chain index
    _nextDesc = 0;

    // Send the configuration to the DMA control registers
    status = P7142InitDmaRegs(&_p7142._p7142DmaParams,
    		&(_p7142._p7142Regs.BAR0RegAddr));
    if (status != 0) {
        ELOG << "Error " << status << 
          " initializing DMA registers for channel " << _chanId;
    }

}

////////////////////////////////////////////////////////////////////////////////
// This static method is called by WinDriver each time a DMA descriptor 
// transfer is completed by an ADC channel (DMA channels 0-3).
void 
p7142Dn::_staticDmaHandler(
        PVOID dmaHandle,
        unsigned int dmaChannel,
        PVOID pData,
        PTK714X_INT_RESULT *pIntResult)
{
    ///if (pIntResult->intLost > 0) {
    ///    DLOG << "On channel " << dmaChannel << " w/intLost = " <<
    ///    pIntResult->intLost << ", flag is 0x" << std::hex <<
    ///    pIntResult->intFlag << std::dec;
    ///}
    // Cast the user data to p7142Dn*
    p7142Dn * downconverter = static_cast<p7142Dn *>(pData);

    // Call the dmaInterrupt member function in the p7142Dn object
    downconverter->_dmaInterrupt();
}

////////////////////////////////////////////////////////////////////////////////////////
void
p7142Dn::_dmaInterrupt() {
    boost::lock_guard<boost::mutex> lock(_bufMutex);
    std::ostringstream msgStream;
    
    // Find out which DMA descriptor buffer is currently being written. We can 
    // read everything up to that buffer.
    uint32_t currDmaDesc = 0;
    PCI7142_GET_DMA_CURR_XFER_CNTR_CURR_DESCPRT(
            _p7142._p7142Regs.BAR0RegAddr.dmaCurrXferCounter[_chanId], currDmaDesc);
    
    if (currDmaDesc == _nextDesc) {
        ELOG << "ERROR! DMA overrun on channel " << _chanId << 
                ". Raising signal SIGUSR2.";
        // Raise the SIGUSR2 signal, which can be caught by the main program
        // to initiate a restart if desired.
        raise(SIGUSR2);
        return;
    }

    // Read up to the descriptor buffer currently being written via DMA. When 
    // things are running smoothly, we'll just read one buffer here, but 
    // occasionally we may need to play catch up. As long as we are 3 or fewer 
    // buffers behind, we should be OK...
    int nBufsRead = 0;
    while (_nextDesc != currDmaDesc) {
        // Make sure we have a buffer available in _freeBuffers
        if (_freeBuffers.empty()) {
            msgStream << "Dropping data on channel " << _chanId << 
                ", no free buffers (of " << N_READYFLOW_DMA_BUFFERS << 
                ") available!";
            ELOG << msgStream.str();
            break;
        }

        // Get the next free buffer and copy the data from DMA to the buffer
        char * buf = _freeBuffers.front();
        _freeBuffers.pop();
        memcpy(buf, (char*)_dmaBuf[_nextDesc].usrBuf, _DmaDescSize);
        nBufsRead++;
        
        // Add the buffer to the filled buffers queue
        _filledBuffers.push(buf);
    
        // Use the condition variable to tell data consumer (i.e. _read())
        // that new data are available.
        _dataReadyCondition.notify_one();
    
        // Move to the next descriptor in the DMA chain
        _nextDesc = (_nextDesc + 1) % 4;
    }
    
//    // Whine a little if we read more than one buffer in this call.
//    if (nBufsRead > 1) {
//        msgStream.clear();
//        msgStream << "On channel " << _chanId << ", read " << nBufsRead <<
//            " DMA buffers at once (warning only)";
//        WLOG << msgStream.str();
//    }

}

////////////////////////////////////////////////////////////////////////////////////////
int
p7142Dn::_read(char* buf, int bytes) {

    // this is where it all happens

    // _readBuf has room for 2*_DmaDescSize, so a read request
    // cannot ask for more than half of this, since we may need to
    // append one of the data blocks from the circular buffer list
    // to fillBuffers.
    assert(bytes <= _DmaDescSize);

    // If we need more data, grab a buffer from _filledBuffers, waiting
    // for it if necessary.
    while (_readBufAvail < bytes) {
        // move unconsumed data to the front of _readBuf
        char * rbData = &_readBuf[0];
        memmove(rbData, rbData + _readBufOut, _readBufAvail);
        _readBufOut = 0;

        // Block until we have at least one filled buffer available
        // Note that unique_lock releases the lock when it goes out of scope.
        boost::unique_lock<boost::mutex> lock(_bufMutex);
        while (_filledBuffers.size() == 0) {
            _dataReadyCondition.wait(lock);
        }
        // assert that we will not overrun _readBuf
        assert(_readBufAvail + _DmaDescSize <= 2 * _DmaDescSize);
        
        char *buf = _filledBuffers.front();
        _filledBuffers.pop();
        
        // Copy the next filled buffer into _readBuf
        memcpy(rbData + _readBufAvail, buf, _DmaDescSize);
        _readBufAvail += _DmaDescSize;
        
        // Put the buffer back on the free list
        _freeBuffers.push(buf);
    }

    // copy requested bytes from _readBuf[chan] to the user buffer
    char * rbData = &_readBuf[0] + _readBufOut;
    memcpy(buf, rbData, bytes);

    _readBufOut   += bytes;
    _readBufAvail -= bytes;

    // assert that we don't have a math logic error
    assert(_readBufAvail >=0);

    return bytes;
}

////////////////////////////////////////////////////////////////////////////////////////
void
p7142Dn::_start() {

    if (_adcActive) {
      ELOG << __FILE__ << ":" << __FUNCTION__ << ":" << 
        " trying to start ADC channel " << _chanId <<
        " when it is already active";
      return;
    }


    // apply the parameters to the DMA registers for this DMA channel
    P7142DmaChanInit(&(_p7142._p7142DmaParams.dmaChan[_chanId]),
                     &(_p7142._p7142Regs.BAR0RegAddr),
                     _chanId);

    // enable the DMA interrupt, on descriptor finish.
    // _dmaHandlerData contains a pointer to "this", as well
    // as other details.
    int status = PTK714X_DMAIntEnable(_dmaHandle,
                                   PTK714X_DMA_DESCRIPTOR_FINISH,
                                   this,
                                   (PTK714X_INT_HANDLER)_staticDmaHandler);

    if (status != PTK714X_STATUS_OK) {
      ELOG << __FILE__ << ":" << __FUNCTION__ << 
        ": Unable to enable DMA interrupt for channel " << _chanId;
      raise(SIGINT);
    }

    DLOG << "DMA interrupt enabled for channel " << _chanId;

    // Enable the DMA. Transfers will not occur however until the GateFlow
    // FIFOs start receiving data, which will take place when the sd3c
    // timers are started.
    P7142DmaStart(&(_p7142._p7142Regs.BAR0RegAddr), _chanId);

    // Mark this channel as active.
    _adcActive = true;
}

////////////////////////////////////////////////////////////////////////////////////////
void
p7142Dn::_stop() {
    DLOG << "Halting DMA for card " << _p7142._cardIndex << "/channel " << 
      _chanId << " and freeing resources";
    if (_p7142.isSimulating()) {
        return;
    }

    if (!_adcActive) {
        return;
    }

    int status;

    /* Abort any existing transfers */
    P7142DmaAbort(&(_p7142._p7142Regs.BAR0RegAddr), _chanId);

    /* Disable DMA Interrupt for this channel */
    status = PTK714X_DMAIntDisable(_dmaHandle);
    if (status != PTK714X_STATUS_OK) {
      ELOG << __FILE__ << ":" << __FUNCTION__ <<
        ": DMA interrupt disable failed";
    }
    
    for (int desc = 0; desc < 4; desc++) {
      status = PTK714X_DMAFreeMem(_dmaHandle, &_dmaBuf[desc]);
        if (status != PTK714X_STATUS_OK) {
          ELOG << __FILE__ << ":" << __FUNCTION__ <<
            ": DMA memory free failed";
        }
    }

    status = PTK714X_DMAClose(_p7142._deviceHandle, _dmaHandle);
    if (status != PTK714X_STATUS_OK) {
      ELOG << __FILE__ << ":" << __FUNCTION__ <<
        ": DMA channel close failed";
    }

    _adcActive = false;

    DLOG << "DMA terminated for ADC channel " << _chanId;

}

