#include "p716xDn.h"

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
LOGGING("p716xDn");

using namespace Pentek;

////////////////////////////////////////////////////////////////////////////////
p716xDn::p716xDn(
        p716x* p716x,
        uint16_t chanId,
        uint32_t dmaDescSize,
        int decimation,
        int simWaveLength,
        bool sim4bytes,
        bool internalClock) :
  _p716x(*p716x),
  _chanId(chanId),
  _adcChanParams(),
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

    // Set up the ADC
    _setAdcParams();
    
    // Start DMA
    _start();
}

////////////////////////////////////////////////////////////////////////////////
p716xDn::~p716xDn() {
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
p716xDn::read(char* buf, int bufsize) {
    boost::recursive_mutex::scoped_lock guard(_mutex);

    // Enforce that reads are a multiple of 4 bytes, since the Pentek driver
    // (silently) does this, e.g., it will return 4 bytes if 7 are requested,
    // or 0 bytes if 1 is requested.
    // If we are to support other read sizes, we'll have to be a lot smarter, 
    // and keep around a buffer of up to 3 unconsumed bytes between calls here. 
    // For now, we are not that smart...
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
p716xDn::_simulatedRead(char* buf, int bufsize) {
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
p716xDn::bytesRead() {
    //boost::recursive_mutex::scoped_lock guard(_mutex);
    long retval = _bytesRead;
    _bytesRead = 0;
    return retval;
}

////////////////////////////////////////////////////////////////////////////////////////
void
p716xDn::_setAdcParams() {
    // Initialize the ADC channel parameter table to working default values
    P716xSetAdcDefaults(&_p716x._boardResource, &_adcChanParams);
    
    // Have the ADC put its data into the "user block", so that it will go 
    // to our DDC running on the FPGA before being packed for output.
    _adcChanParams.dataSource = P716x_ADC_DATA_CTRL_USR_DATA_SEL_USER;
    
    // Set ADC data packing mode to deliver I and Q data
    _adcChanParams.dataPackMode = P716x_ADC_DATA_CTRL_PACK_MODE_IQ_DATA_PACK;
    
    DWORD uStatus;  // unsigned
    int status;     // signed
    
    // Open a DMA channel
    uStatus = PTK716X_DMAOpen(_p716x._deviceHandle, _chanId, &_dmaHandle);
    if (uStatus != PTK716X_STATUS_OK) {
        ELOG << __PRETTY_FUNCTION__ <<
                ": Unable to open DMA for ADC channel " << _chanId;
        abort();
    }

    // Allocate DMA data buffers
    for (int d = 0; d < N_DMA_DESCRIPTORS; d++) {
        uStatus = PTK716X_DMAAllocMem(_dmaHandle, _DmaDescSize, &_dmaBuf[d], TRUE);
        if (uStatus != PTK716X_STATUS_OK) {
            ELOG << __PRETTY_FUNCTION__ << 
            ": Unable to allocate a DMA buffer for ADC channel " << _chanId << 
            "/descriptor " << d;
            abort();
        }
    }

    // Apply our ADC parameter table to the registers
    status = P716xInitAdcRegs(&_adcChanParams, &_p716x._regAddr, _chanId);
    if (status != 0) {
        ELOG << __PRETTY_FUNCTION__ << ": Bad status " << status << 
                " from P716xInitAdcRegs() for ADC channel " << _chanId;
        abort();
    }
    
    // DMA setup -------------------------------------------------------

    // reset the DMA linked list engine and FIFO
    P716xAdcDmaReset(&_p716x._regAddr, _chanId);

    // Build a DMA descriptor chain to loop in a circle through 
    // N_DMA_DESCRIPTORS descriptors.
    P716x_ADC_DMA_LLIST_DESCRIPTOR descriptorParams;
    P716x_ADC_DMA_DESCRIPT_CWORD_PARAMS cwordParams;
    
    for (int d = 0; d < N_DMA_DESCRIPTORS; d++) {
        // Set the DMA transfer length
        descriptorParams.xferLength = _DmaDescSize;
        
        // Set the memory addresses for DMA transfers from this descriptor
        P716xSetAdcDmaLListDescriptorAddress(_dmaBuf[d].kernBuf,
                                             &descriptorParams.mswAddress,
                                             &descriptorParams.lswAddress);
        
        // Put default values into the control word params table
        P716xSetAdcDmaLListDescriptCwordDefaults(&cwordParams);
        
        // Next descriptor will be current index + 1, with a loop back to zero 
        // if we're at the end of the list.
        cwordParams.nextLinkIndx = (d + 1) % N_DMA_DESCRIPTORS;
        
        // Generate an interrupt when this link's DMA completes
        cwordParams.linkEndIntr = P716x_ADC_DMA_CWORD_LINK_END_INTR_ENABLE;
        
        // Use DMA auto start mode
        cwordParams.startMode = P716x_ADC_DMA_CWORD_START_MODE_AUTO;
        
        // Apply the control word parameters table to set the linked-list
        // descriptor control word.
        descriptorParams.linkCtrlWord = 
                P716xBuildAdcDmaLListDescriptCword(&cwordParams);
        
        // Finally, write the linked-list descriptor parameters into the DMA 
        // linked-list RAM.
        P716xInitAdcDmaLListDescriptor(&descriptorParams,
                                       &_p716x._regAddr, 
                                       _chanId, 
                                       d);
    }

    // Flush the ADC Input FIFOs
    P716xAdcFifoFlush(&_p716x._regAddr, _chanId);

    // reset the DMA linked list engine
    P716xAdcDmaReset(&_p716x._regAddr, _chanId);

    // arm the DMA for a trigger
    P716xAdcDmaStart(&_p716x._regAddr, _chanId);
    
    // Initialize our DMA chain index
    _nextDesc = 0;
}

////////////////////////////////////////////////////////////////////////////////
// This static method is called by WinDriver each time a DMA descriptor 
// transfer is completed by an ADC channel (DMA channels 0-3).
void 
p716xDn::_staticDmaHandler(
        PVOID deviceHandle,
        unsigned int intType,
        PVOID instancePointer,
        PTK716X_INT_RESULT *intResult)
{
    // Cast instancePointer to p716xDn*
    p716xDn * downconverter = static_cast<p716xDn *>(instancePointer);

    // Call the dmaInterrupt member function in the p716xDn object
    downconverter->_dmaInterrupt();
}

////////////////////////////////////////////////////////////////////////////////////////
void
p716xDn::_dmaInterrupt() {
    boost::lock_guard<boost::mutex> lock(_bufMutex);
    std::ostringstream msgStream;
    
    // Find out which DMA descriptor buffer is currently being written. We can 
    // read everything up to that buffer.
    uint32_t currDmaDesc = 0;
    PCI716x_GET_DMA_CURR_XFER_CNTR_CURR_DESCPRT(
            _p716x._p716xRegs.BAR0RegAddr.dmaCurrXferCounter[_chanId], currDmaDesc);
    
    if (currDmaDesc == _nextDesc) {
        msgStream << "ERROR! Channel " << _chanId << 
            " wants to read descriptor " << _nextDesc << 
            " while DMA is in progress there! Likely overrun!\n";
        ELOG << msgStream.str();
        // Skip reading this descriptor, since the old data we want is being
        // overwritten right now!
        _nextDesc = (_nextDesc + 1) % 4;
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
                ", no free buffers available!\n";
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
    
    // Whine a little if we read more than one buffer in this call.
    //if (nBufsRead > 1) {
        //msgStream.clear();
        //msgStream << "On channel " << _chanId << ", read " << nBufsRead <<
        //    " DMA buffers at once (warning only)";
        //ELOG << msgStream.str();
    //}

}

////////////////////////////////////////////////////////////////////////////////////////
int
p716xDn::_read(char* buf, int bytes) {

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
p716xDn::_start() {

    if (_adcActive) {
      ELOG << __FILE__ << ":" << __FUNCTION__ << ":" << 
        " trying to start ADC channel " << _chanId <<
        " when it is already active";
      return;
    }


    // apply the parameters to the DMA registers for this DMA channel
    P716xDmaChanInit(&(_p716x._p716xDmaParams.dmaChan[_chanId]),
                     &(_p716x._p716xRegs.BAR0RegAddr),
                     _chanId);

    // enable the DMA interrupt, on descriptor finish.
    // _dmaHandlerData contains a pointer to "this", as well
    // as other details.
    int status = PTK716X_intEnable(moduleResrc->hDev,
                                   PTK716X_PCIE_INTR_ADC_ACQ_MOD1 << _chanId,
                                   P716x_ADC_INTR_LINK_END,
                                   this,
                                   _staticIntHandler);

    int status = PTK716X_DMAIntEnable(_dmaHandle,
                                   PTK716X_DMA_DESCRIPTOR_FINISH,
                                   this,
                                   (PTK716X_INT_HANDLER)_staticDmaHandler);

    if (status != PTK716X_STATUS_OK) {
      ELOG << __FILE__ << ":" << __FUNCTION__ << 
        ": Unable to enable DMA interrupt for channel " << _chanId;
      raise(SIGINT);
    }

    DLOG << "DMA interrupt enabled for channel " << _chanId;

    // Enable the DMA. Transfers will not occur however until the GateFlow
    // FIFOs start receiveing data, which will take place when the sd3c
    // timers are started.
    P716xDmaStart(&(_p716x._p716xRegs.BAR0RegAddr), _chanId);

    // Mark this channel as active.
    _adcActive = true;
}

////////////////////////////////////////////////////////////////////////////////////////
void
p716xDn::_stop() {
    DLOG << "Halting DMA for card " << _p716x._cardIndex << "/channel " << 
      _chanId << " and freeing resources";
    if (_p716x.isSimulating()) {
        return;
    }

    if (!_adcActive) {
        return;
    }

    int status;

    // Abort any existing transfers
    P716xAdcDmaAbort(&_p716x._regAddr, _chanId);

    // Disable DMA Interrupt for this channel
    status = PTK716x_DMAIntDisable(_dmaHandle);
    if (status != PTK716X_STATUS_OK) {
      ELOG << __FILE__ << ":" << __FUNCTION__ <<
        ": DMA interrupt disable failed";
    }
    
    for (int desc = 0; desc < 4; desc++) {
      status = PTK716X_DMAFreeMem(_dmaHandle, &_dmaBuf[desc]);
        if (status != PTK716X_STATUS_OK) {
          ELOG << __FILE__ << ":" << __FUNCTION__ <<
            ": DMA memory free failed";
        }
    }

    status = PTK716X_DMAClose(_p716x._deviceHandle, _dmaHandle);
    if (status != PTK716X_STATUS_OK) {
      ELOG << __FILE__ << ":" << __FUNCTION__ <<
        ": DMA channel close failed";
    }

    _adcActive = false;

    DLOG << "DMA terminated for ADC channel " << _chanId;

}

