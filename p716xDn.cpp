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
#include <sys/syscall.h>    // to use SYS_gettid
#include <sstream>

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
  _ifcArgs(),
  _DmaDescSize(dmaDescSize),
  _bytesRead(0),
  _simWaveLength(simWaveLength),
  _angleCount(0),
  _sim4bytes(sim4bytes),
  _mutex(),
  _adcActive(false),
  _readBufAvail(0),
  _readBufOut(0),
  _exiting(false)
{
    // dma buffer size must be a multiple of 4
    if ((_DmaDescSize % 4) || (_DmaDescSize <= 0)) {
      ELOG << "DMA descriptor size must be a positive multiple of 4 bytes, "
           <<  _DmaDescSize << " was specified";
      raise(SIGINT);
    }

    boost::recursive_mutex::scoped_lock guard(_mutex);

    // Initialize ReadyFlow OS-dependent resources
    if (PTKIFC_Init(&_ifcArgs) != 0) {
        ELOG << "Error initializing Pentek IFC_ARGS for channel " << _chanId;
        raise(SIGINT);
        return;
    }
    
    if (isSimulating()) {
        _adcActive = true;
        return;
    }

    // create a capture file
#ifdef DMA_FILE_CAPTURE
    std::stringstream fname;
    fname <<  "sd3cdn_" << _chanId << ".dat";
    _captureFile.open(fname.str().c_str(), std::ios::out | std::ios::binary);
#endif
    // Size _readBuf appropriately.
    _readBuf.resize(2 * _DmaDescSize);
    
    // Initialize ADC parameters for our input channel with default
    // working values.
    P716xSetAdcDefaults(&_p716x._boardResource, &_adcParams);

    // Apply the ADC parameters to the ADC registers on the card
    _applyAdcParams();
    
    // Create our DMA reader thread
    _initDma();

    // Start DMA
    _start();
}

////////////////////////////////////////////////////////////////////////////////
p716xDn::~p716xDn() {
    boost::recursive_mutex::scoped_lock guard(_mutex);
    // Set _exiting to true to terminate the DMA reading thread, then wait
    // for the thread to complete.
    ILOG << "Waiting for DMA reading thread to finish";
    _exiting = true;
    PTKIFC_ThreadWaitFinish(&_ifcArgs, _dmaThreadNum());

    // Stop DMA
    _stop();
    
    // Free DMA resources
    int status;
    for (int d = 0; d < N_DMA_DESCRIPTORS; d++) {
        status = PTK716X_DMAFreeMem(_dmaHandle, &_dmaBuf[d]);
        if (status != PTK716X_STATUS_OK) {
            ELOG << __PRETTY_FUNCTION__ <<
                    ": DMA memory free failed for channel " << _chanId <<
                    "/descriptor " << d;
        }
    }

    status = PTK716X_DMAClose(_p716x._deviceHandle, _dmaHandle);
    if (status != PTK716X_STATUS_OK) {
        ELOG << __PRETTY_FUNCTION__ << ": DMA channel close failed";
    }

    // Clean up Pentek mutex/thread information
    PTKIFC_UnInit(&_ifcArgs);

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
#ifdef DMA_FILE_CAPTURE
    _captureFile.close();
#endif
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
      raise(SIGINT);
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

////////////////////////////////////////////////////////////////////////////////
void
p716xDn::_initDma() {
    // Put a set of _DmaDescSize char buffers in the free buffer queue
    // for this channel. These buffers are used when pulling data 
    // from DMA.
    for (int i = 0; i < N_READYFLOW_DMA_BUFFERS; i++) {
        _freeBuffers.push(new char[_DmaDescSize]);
    }
    
    // Open a DMA channel
    DWORD uStatus;  // unsigned
    uStatus = PTK716X_DMAOpen(_p716x._deviceHandle, _chanId, &_dmaHandle);
    if (uStatus != PTK716X_STATUS_OK) {
        ELOG << __PRETTY_FUNCTION__ <<
                ": Unable to open DMA for ADC channel " << _chanId;
        raise(SIGINT);
    }

    // reset the DMA linked list engine and FIFO
    P716xAdcDmaReset(&_p716x._regAddr, _chanId);

    // Build a DMA descriptor chain which circles through N_DMA_DESCRIPTORS 
    // descriptors.
    P716x_ADC_DMA_LLIST_DESCRIPTOR descriptorParams;
    P716x_ADC_DMA_DESCRIPT_CWORD_PARAMS cwordParams;
    
    for (int d = 0; d < N_DMA_DESCRIPTORS; d++) {
        // Allocate DMA memory space for the descriptor
        uStatus = PTK716X_DMAAllocMem(_dmaHandle, _DmaDescSize, &_dmaBuf[d], TRUE);
        if (uStatus != PTK716X_STATUS_OK) {
            ELOG << __PRETTY_FUNCTION__ << 
                    ": Unable to allocate a DMA buffer for ADC channel " << 
                    _chanId << "/descriptor " << d;
            raise(SIGINT);
        }
            
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

    // Create a semaphore used to indicate a DMA buffer has been written.
    // We just use our ADC channel number as the semaphore number.
    if (PTKIFC_SemaphoreCreate (&_ifcArgs, _dmaCompleteSemNum()) < 0) {
        ELOG << __PRETTY_FUNCTION__ << 
                ": Unable to create a 'DMA complete' semaphore for ADC " << 
                _chanId;
        raise(SIGINT);
    }

    // Bogus cast necessary because the signature for PTKIFC_ThreadCreate
    // wants a threadFunc which takes no arguments, even though the function is 
    // actually called with a single void* argument. If they ever fix this, we
    // can just pass _StaticDmaThreadMainLoop as the 3rd argument below.
    void (*threadFunc)(void) = 
            reinterpret_cast<void (*)(void)>(_StaticDmaThreadMainLoop);
    
    // Create/start our DMA reader thread. The thread executes static method
    // _StaticDmaThreadMainLoop(this), which in turn executes 
    // this->_dmaThreadMainLoop().
    DWORD status = PTKIFC_ThreadCreate(&_ifcArgs, _dmaThreadNum(), threadFunc, this);
    if (status != 0)
    {
        ELOG << __PRETTY_FUNCTION__ << 
                ": PTKIFC_ThreadCreate failed with status " << status << 
                " for ADC channel " << _chanId;
        raise(SIGINT);
    }
}

////////////////////////////////////////////////////////////////////////////////
void
p716xDn::_applyAdcParams() {
    // Apply our ADC parameter table to the registers
    int status = P716xInitAdcRegs(&_adcParams, &_p716x._regAddr, _chanId);
    if (status != 0) {
        ELOG << __PRETTY_FUNCTION__ << ": Bad status " << status << 
                " from P716xInitAdcRegs() for ADC channel " << _chanId;
        raise(SIGINT);
    }
}

////////////////////////////////////////////////////////////////////////////////////////
void
p716xDn::_dmaSemaphorePost() {
    if (PTKIFC_SemaphorePost(&_ifcArgs, _dmaCompleteSemNum()) < 0) {
        ELOG << "Error posting 'DMA complete' semaphore for channel " << _chanId;
    }
}

////////////////////////////////////////////////////////////////////////////////////////
void
p716xDn::_dmaThreadMainLoop() {
    // Print the id for this thread
    DLOG << "DMA thread for channel " << _chanId << " has ID " <<
            syscall(SYS_gettid) << ", and uses semaphore " << _dmaCompleteSemNum();
    // Loop until _exiting is set to true by the destructor
    while (! _exiting) {
        // Wait up to WAIT_MSECS ms for the 'DMA complete' semaphore
        static const int WAIT_MSECS = 1000;
        bool fail = PTKIFC_SemaphoreWait(&_ifcArgs, _dmaCompleteSemNum(),
                                         IFC_WAIT_STATE_MILSEC(WAIT_MSECS));
        if (fail) {
            DLOG << "No DMA completion in " << WAIT_MSECS <<
                    " ms on channel " << _chanId;
            continue;
        }

        boost::lock_guard<boost::mutex> lock(_bufMutex);

        // Make sure we have a buffer available in _freeBuffers
        if (_freeBuffers.empty()) {
            ELOG << "Dropping data on channel " << _chanId << 
                ", no free buffers available!";
            continue;
        }

        // Get the next free buffer and copy the data from DMA to the buffer
        char * buf = _freeBuffers.front();
        _freeBuffers.pop();
        memcpy(buf, (char*)_dmaBuf[_nextDesc].usrBuf, _DmaDescSize);
        
        // Capture the new data
#ifdef DMA_FILE_CAPTURE
        _captureFile.write((char*)_dmaBuf[_nextDesc].usrBuf, _DmaDescSize);
#endif
        // Add the buffer to the filled buffers queue
        _filledBuffers.push(buf);
    
        // Use the condition variable to tell data consumer (i.e. _read())
        // that new data are available.
        _dataReadyCondition.notify_one();
        
        // Increment the next descriptor to read
        _nextDesc = (_nextDesc + 1) % N_DMA_DESCRIPTORS;
    }
    
    PTKIFC_ThreadExit(&_ifcArgs);
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
        ELOG << __PRETTY_FUNCTION__ << ": trying to start ADC channel " << 
                _chanId << " when it is already active";
        return;
    }

    // Flush the ADC Input FIFOs
    P716xAdcFifoFlush(&_p716x._regAddr, _chanId);

    // Reset the DMA linked list engine
    P716xAdcDmaReset(&_p716x._regAddr, _chanId);
    
    // We'll start reading from the first DMA descriptor
    _nextDesc = 0;

    // enable the DMA 'Link End' interrupt for this channel
    if (PTKIFC_MutexLock(&_ifcArgs, 0, IFC_WAIT_STATE_MILSEC(100)) != 0) {
        ELOG << "Unable to lock mutex to enable 'Link End' interrupts on chan" <<
                _chanId;
        raise(SIGINT);
        return;
    }
    DWORD uStatus = PTK716X_intEnable(_p716x._deviceHandle,
                                      (PTK716X_PCIE_INTR_ADC_ACQ_MOD1 << _chanId),
                                      P716x_ADC_INTR_LINK_END,
                                      this,
                                      _StaticIntHandler);
    PTKIFC_MutexUnlock(&_ifcArgs, 0);
    if (uStatus == PTK716X_STATUS_OK) {
    	DLOG << "Enabled 'Link End' interrupt for ADC " << _chanId;
    } else {
        ELOG << __PRETTY_FUNCTION__ << 
                ": Could not enable 'Link End' interrupt for ADC " << _chanId;
        raise(SIGINT);
    }

    // Flush the CPU and I/O caches for each DMA descriptor's buffer
    for (int i = 0; i < N_DMA_DESCRIPTORS; i++) {
        PTK716X_DMASyncCpu(&_dmaBuf[i]);
        PTK716X_DMASyncIo(&_dmaBuf[i]);
    }

    // clear any pending ADC interrupt flags for our channel
    P716x_REG_WRITE(_p716x._regAddr.adcRegs[_chanId].interruptFlag,
                    P716x_ADC_INTR_REG_MASK);

    // Enable the DMA. Transfers will not occur however until the GateFlow
    // FIFOs start receiving data, which will take place when the sd3c
    // timers are started.
    P716xAdcDmaStart(&_p716x._regAddr, _chanId);

    // Mark this channel as active.
    _adcActive = true;
}

////////////////////////////////////////////////////////////////////////////////////////
void
p716xDn::_stop() {
    DLOG << "Halting DMA for card " << _p716x._cardIndex << 
            "/channel " << _chanId;
    if (_p716x.isSimulating()) {
        return;
    }

    if (!_adcActive) {
        return;
    }

    // Abort any existing transfers
    P716xAdcDmaAbort(&_p716x._regAddr, _chanId);

    // Disable the DMA 'Link End' interrupt for this channel
    DWORD uStatus = PTK716X_intDisable(_p716x._deviceHandle,
                                       (PTK716X_PCIE_INTR_ADC_ACQ_MOD1 << _chanId),
                                       P716x_ADC_INTR_LINK_END);
    if (uStatus != PTK716X_STATUS_OK)
    {
        ELOG << __PRETTY_FUNCTION__ << 
                ": Could not disable 'Link End' interrupt for ADC " << _chanId;
        raise(SIGINT);
    }

    DLOG << "DMA terminated for ADC channel " << _chanId;
}

