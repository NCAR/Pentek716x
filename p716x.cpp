#include "p716x.h"
#include "p716xDn.h"
//#include "p716xUp.h"
#include <fcntl.h>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <csignal>

#include <logx/Logging.h>
LOGGING("p716x");

using namespace Pentek;

////////////////////////////////////////////////////////////////////////////////////////

/// Semaphore used by the _ddrMemWrite() routine to signal that an interrupt was
/// received from a dma transfer
sem_t ddrMemWriteSem;

/// Interrupt handler used by the ddrMemWrite() DMA transfers
void ddrMemWriteIntHandler(PVOID               hDev,
                        int                lintSource,
                        PVOID               pData,
                        PTK716X_INT_RESULT *pIntResult)
{
    sem_post(&ddrMemWriteSem);
}

////////////////////////////////////////////////////////////////////////////////////////

/// Semaphore used by the _ddrMemRead() routine to signal that an interrupt was
/// received from a dma transfer.
sem_t ddrMemReadSem;

/// Interrupt handler used by the _ddrMemRead() DMA transfers.
void ddrMemReadIntHandler(PVOID               hDev,
                       int                lintSource,
                       PVOID               pData,
                       PTK716X_INT_RESULT *pIntResult)
{
    sem_post(&ddrMemReadSem);
}

/// Static member to keep track of the "PCI slot" of the last instantiated p716x.
/// This is needed for calls to ReadyFlow's PTK716X_DeviceFindAndOpen() 
/// function. To open 716x cards in sequence, we start with _Next716xSlot = -2.
/// See the ReadyFlow documentation for PTK716X_DeviceFindAndOpen().
DWORD p716x::_Next716xSlot = -2;

/// Static member to keep track of how many 716x cards we have open (i.e., how
/// many instances of this class are there so far).
uint16_t p716x::_NumOpenCards = 0;

////////////////////////////////////////////////////////////////////////////////////////
p716x::p716x(double clockFrequency, bool useInternalClock, bool useFirstCard,
             bool simulate, double simPauseMS):
_clockFrequency(clockFrequency),
_useInternalClock(useInternalClock),
_cardIndex(_NumOpenCards),
_moduleId(P71620_MODULE_ID),    // 71620 is the only one we support right now
_simulate(simulate),
_p716xMutex(),
_isReady(false),
_upconverter(0),
_simPulseNum(0),
_waitingDownconverters(0),
_simWaitCounter(0),
_simPauseMS(simPauseMS)
{
	boost::recursive_mutex::scoped_lock guard(_p716xMutex);

	// If we're simulating, things are simple...
    if (_simulate) {
        _isReady = true;
    } else {
    	// initialize ReadyFlow
    	if (useFirstCard) {
    		// we have been explicitly told to find the first card.
    		_Next716xSlot = -2;
    	}
    	_isReady = _initReadyFlow();
    }
    // If we were successful, increment the open card count
    if (_isReady)
        _NumOpenCards++;

    return;
}

////////////////////////////////////////////////////////////////////////////////
p716x::~p716x() {
    if (_simulate) {
        return;
    }

    boost::recursive_mutex::scoped_lock guard(_p716xMutex);

    // destroy the down converters
    for (std::map<int, DownconverterInfo>::iterator i = _downconverters.begin();
    	i != _downconverters.end(); i++) {
        delete i->second._dn;
    }
    // empty the list of active down converters
    _downconverters.erase(_downconverters.begin(), _downconverters.end());

    /* cleanup for exit */
    PTK716X_DeviceClose(_deviceHandle);
    _NumOpenCards--;
    
    /* If there are no instances left, close up the ReadyFlow library */
    if (_NumOpenCards == 0) {
        PTK716X_LibUninit();
        DLOG << "ReadyFlow closed";
    }
}

////////////////////////////////////////////////////////////////////////////////
p716xDn*
p716x::addDownconverter(int chanId, uint32_t dmaDescSize, int bypassdivrate,
        int simWavelength, bool sim4bytes) {
    boost::recursive_mutex::scoped_lock guard(_p716xMutex);

    // Just construct a new downconverter and put it in our list.
    p716xDn* downconverter = new p716xDn(
    		this,
    		chanId,
    		dmaDescSize,
    		bypassdivrate,
            simWavelength,
            sim4bytes);
    _addDownconverter(downconverter);
    return(downconverter);
}

////////////////////////////////////////////////////////////////////////////////
p716xUp*
p716x::addUpconverter(
		double sampleClockHz,
        double ncoFreqHz,
        char mode) {
    boost::recursive_mutex::scoped_lock guard(_p716xMutex);
    // Just construct a new upconverter and put it in our list.
    p716xUp* upconverter = new p716xUp(
    		this,
    		sampleClockHz,
    		ncoFreqHz,
    		mode);
    _addUpconverter(upconverter);
    return(upconverter);
}

////////////////////////////////////////////////////////////////////////////////////////
int p716x::memWrite(int bank, int32_t* buf, int bytes) {

    int retval = bytes;

    int status = _ddrMemWrite(
            &_boardResource,
            bank,
            bytes,
            (uint32_t*)buf,
            _deviceHandle);

    if (status != 0) {
        std::cout << "memory write failed, ddrMemWrite returned " << status << std::endl;
        retval = -1;
    }

    return retval;
}

////////////////////////////////////////////////////////////////////////////////////////
int p716x::memRead(int bank, int32_t* buf, int bytes) {

    int retval = bytes;

    int status = _ddrMemRead(
            &_boardResource,
            bank,
            bytes,
            (uint32_t*)buf,
            _deviceHandle);

    if (status != 0) {
        std::cout << "memory read failed, ddrMemRead returned " << status << std::endl;
        retval = -1;
    }

    return retval;


    return 0;
}

////////////////////////////////////////////////////////////////////////////////////////
bool
p716x::_initReadyFlow() {

    _deviceHandle = NULL;

    // initialize the PTK716X library
    DWORD dwStatus = PTK716X_LibInit();
    if (dwStatus != PTK716X_STATUS_OK)
    {
      ELOG << "Failed to initialize the PTK716X library.";
      ELOG << "Make sure that windrvr6 is loaded (did you do a kernel upgrade recently?).";
      ELOG << "Also make sure you have read permission on /dev/windrvr6.";
      return false;
    }

    // Find and open the next PTK716X device
    _deviceHandle = PTK716X_DeviceFindAndOpen(&_Next716xSlot, &_BAR0Base, 
            &_BAR2Base, &_BAR4Base, _moduleId);
    if (_deviceHandle == NULL)
    {
      ELOG << "Pentek 716x device not found when opening card " << 
        _NumOpenCards;
    }

    // Reset the board so we start in pristine condition
    P716xSetGlobalResetState(_regAddr.globalReset, P716x_GLOBAL_RESET_ENABLE);
    usleep(P716X_IOCTLSLEEPUS);
    P716xSetGlobalResetState(_regAddr.globalReset, P716x_GLOBAL_RESET_DISABLE);
    usleep(P716X_IOCTLSLEEPUS);

    // Initialize 716x register address tables
    P716xInitRegAddr(_BAR0Base, &_regAddr, &_boardResource, _moduleId);

    // Reset board registers to power-on default states
    P716xResetRegs(&_regAddr);
    
    // Load parameter tables with default values
    P716xSetGlobalDefaults(&_boardResource, &_globalParams);

    // check if module is a 716x
    unsigned int id = P716xGetModuleId(_regAddr.boardId);
    if (id != _moduleId)
    {
        ELOG << "Module id of Pentek card " << _NumOpenCards + 1 << " is " <<
                std::hex << id << ", but a " << _moduleId << " is expected";
        return false;
    }

    DLOG << "Pentek 716x device";
    DLOG << std::hex << " BAR0: 0x" << (void *)_BAR0Base;
    DLOG << std::hex << " BAR2: 0x" << (void *)_BAR2Base;
    DLOG << std::hex << " BAR4: 0x" << (void *)_BAR4Base;

    // Verify that the board has the Pentek DDC IP core
    if (_boardResource.numDDC == 0) {
        ELOG << "P716x card " << _NumOpenCards << 
                " does not have the Pentek DDC IP core installed!";
        return false;
    }
    
    // Get the per-channel DDC register addresses
    P716xInitDdcRegAddr(_BAR2Base, &_ddcRegAddr);
    
    /// @todo Although we follow the normal ReadyFlow protocol
    /// for configuring the DAC (P716xSetDac5687Defaults()
    /// followed by P716xInitDac5687Regs()),
    /// the DAC is completely reconfigured in p716xUp().
    /// We need to modify P716xSetDac5687Defaults() and
    /// P716xInitDac5687Regs() to perform the correct configuration,
    /// so that it can be pulled out of p716xUp().

    // Make adjustments to various register configuration parameters
    _configClockParameters();

    // Validate parameters before applying them
    if (P716xValidateBrdClkFreq(&_globalParams, &_regAddr) != 0) {
        ELOG << "ADC or DAC sampling frequency is incompatible with board " <<
                "clock frequency of " << 1.0e-6 * _clockFrequency << " MHz";
        return false;
    }
    
    // Write parameter table values to the 716x registers
    P716xInitGlobalRegs(&_globalParams, &_regAddr);

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////
void p716x::_configClockParameters() {
    // Set the clock frequency in the global params struct. This is the 
    // frequency of the external source if an external clock is used, or the 
    // frequency for which the card's onboard CDC7005 clock synthesizer will be
    // configured. In turn, by the card's defaults, this will be the clock 
    // frequency provided for both the ADC and DAC channels.
    _globalParams.brdClkFreq = _clockFrequency;
    
    // Set up use of internal or external clock
    if (_useInternalClock) {
        // Enable the onboard VCXO
        _globalParams.sbusParams.vcxoOutput = P716x_SBUS_CTRL1_VCXO_OUT_ENABLE;
        // Have the CDC7005 clock synthesizer use the VCXO as its clock input,
        // providing no reference clock to discipline it.
        _globalParams.sbusParams.clockSelect = P716x_SBUS_CTRL1_CLK_SEL_VCXO_NO_REF;
    } else {
        // Turn off output from the onboard VCXO, since we'll use the external
        // clock.
        _globalParams.sbusParams.vcxoOutput = P716x_SBUS_CTRL1_VCXO_OUT_DISABLE;
        // Have the CDC7005 clock synthesizer just pass the external clock
        // directly through.
        _globalParams.sbusParams.clockSelect = P716x_SBUS_CTRL1_CLK_SEL_EXT_CLK;
    }
}

////////////////////////////////////////////////////////////////////////////////////////
void p716x::_configOutParameters() {

    // Customize the up conversion path

    /// @todo Is the following correct for PLL usage? What do they mean by "bypass"?
    _p716xOutParams.dacClkSel = P716x_DAC_CTRL_STAT_DAC_CLK_BYPASS;
    _p716xOutParams.dacPllVdd = P716x_DAC_CTRL_STAT_PLL_VDD_ENABLE;
    _p716xOutParams.outputSyncBusSel = P716x_SYNC_BUS_SEL_A;
    // Following pack mode is used becasue we are putting I and Q into a 32 bit memory word.
    _p716xOutParams.dacFifo.fifoPackMode = P716x_FIFO_DAC_PACK_MODE_UNPACK;
    _p716xOutParams.dacFifo.fifoWordSwap = P716x_FIFO_CTRL_WORD_SWAP_DISABLE;

    // The FIFO Almost Full and Almost Empty levels are set to default
    // values for all programs.  The values shown here are the default
    // values for DAC FIFOs and are provided to show usage.   Their values
    // must be chosen to work with the DMA channel maximum burst count
    // value.
    _p716xOutParams.dacFifo.fifoAlmostEmptyLevel = 6144;
    _p716xOutParams.dacFifo.fifoAlmostFullLevel  = 6176;
}

////////////////////////////////////////////////////////////////////////////////////////
void
p716x::disableSyncAndGateGen() {
    P716xSetGateGenState(_regAddr.gateAGenerate, P716x_GATE_INACTIVE);
    P716xSetSyncGenState(_regAddr.syncAGenerate, P716x_SYNC_INACTIVE);
    P716xSetGateGenState(_regAddr.gateBGenerate, P716x_GATE_INACTIVE);
    P716xSetSyncGenState(_regAddr.syncBGenerate, P716x_SYNC_INACTIVE);
    DLOG << "Gate and sync generation disabled";
}

////////////////////////////////////////////////////////////////////////////////
void
p716x::_addDownconverter(p716xDn * downconverter) {
    boost::recursive_mutex::scoped_lock guard(_p716xMutex);
    
    int chan = downconverter->chanId();
    if (_downconverters.find(chan) != _downconverters.end()) {
        std::cerr << "Existing downconverter for channel " << chan <<
                " is being replaced" << std::endl;
        delete _downconverters[chan]._dn;
    }
    _downconverters[chan]._dn = downconverter;
    _downconverters[chan]._pulseNum = 0;
}

////////////////////////////////////////////////////////////////////////////////
void
p716x::_addUpconverter(p716xUp * upconverter) {
    boost::recursive_mutex::scoped_lock guard(_p716xMutex);

    if (_upconverter) {
        std::cerr << "Existing upconverter is being replaced"  << std::endl;
        delete _upconverter;
    }
    _upconverter = upconverter;
}

////////////////////////////////////////////////////////////////////////////////////////
void
p716x::_resetDCM() {
    boost::recursive_mutex::scoped_lock guard(_p716xMutex);

    if (isSimulating())
        return;

    // cycle the digital clock manager
    // hold the dcm in reset for a short period
    P716x_SET_DCM_CTRL_DCM_RST(_boardResource.BAR2RegAddr.dcmControl, P716x_DCM_CTRL_DCM_RST_RESET);
    usleep(1000);
    // take the dcm out of reset
    P716x_SET_DCM_CTRL_DCM_RST(_boardResource.BAR2RegAddr.dcmControl, P716x_DCM_CTRL_DCM_RST_RUN);
    usleep(1000);

    DLOG << "DCM has been cycled.";

    return;

}

////////////////////////////////////////////////////////////////////////////////////////
/****************************************************************************
 Function: _ddrMemWrite

 Description: writes data to the selected DDR memory bank.  Uses DMA
              Channel 7.

 Inputs:      p716xRegs     - pointer to the 716x register addres table
              bank          - use defines:
                                  P716x_DDR_MEM_BANK0
                                  P716x_DDR_MEM_BANK1
                                  P716x_DDR_MEM_BANK2
              dataLen       - number bytes to write
              dataBuf       - pointer to the data buffer containing the data
              hDev          - 716x Device Handle

 Returns:     0 - successful
              1 - invalid bank number
              3 - bank depth extends past the end of the DDR bank
              4 - DMA channel failed to open
              5 - DMA buffer allocation failed
              6 - semaphore creation failed
              7 - semaphore wait timed out
              8 - initial memory read failed
              9 - verification of written memory failed
****************************************************************************/
int p716x::_ddrMemWrite(P716x_BOARD_RESOURCE* p716xRegs,
                 unsigned int    bank,
                 unsigned int    dataLen,
                 uint32_t*       dataBuf,
                 PVOID           hDev)
{
    PTK716X_DMA_HANDLE  *ddrDmaWriteHandle;
    PTK716X_DMA_BUFFER   dmaBuf;
    P716x_FIFO_PARAMS    fifoParams;   /* FIFO params */
    P716x_DMA_PARAMS     dmaParams;    /* DMA params */
    P716x_DDR_MEM_PARAMS ddrMemParams; /* DDR memory params */
    unsigned int         bankDepth;
    int                  status;
    uint32_t             writeLenBytes;
    uint16_t             writeLenBlocks;

    // We need to write in 32-byte blocks, and we need to write at least
    // 17 of them (544 bytes) to be consistently successful. The 17 block
    // number was determined empirically, with multiple tests of data
    // writes of different lengths (4-8192 bytes, in 4-byte increments)
    // with readback to verify successful writes.
    writeLenBlocks = (dataLen + 31) / 32 + 1;
    if (writeLenBlocks < 17)
        writeLenBlocks = 17;
    // The number of bytes we'll actually write
    writeLenBytes = writeLenBlocks * 32;
    bankDepth = writeLenBytes / 4;   // bank addresses are in 4-byte words
    assert((bankDepth & 0x7) == 0);

    /* check input parameters -------------------------------------------- */

    /* check bank */
    if ( (bank != P716x_DDR_MEM_BANK0) &&
         (bank != P716x_DDR_MEM_BANK1) &&
         (bank != P716x_DDR_MEM_BANK2)    )
        return (1);

    /* check dataLen */
    if (writeLenBytes > P716x_DDR_MEM_BANK_BYTE_SIZE)
        return (3);


    /* DMA setup --------------------------------------------------------- */

    /* open a DMA channel to write to the delay memory */
    status = PTK716X_DMAOpen(hDev, P716x_DMA_CHAN_7, &ddrDmaWriteHandle);
    if (status != PTK716X_STATUS_OK)
        return (4);

    /* allocate system memory for write data buffer */
    status = PTK716X_DMAAllocMem(ddrDmaWriteHandle, writeLenBytes, &dmaBuf, (BOOL)0);
    if (status != PTK716X_STATUS_OK)
    {
        PTK716X_DMAClose(hDev, ddrDmaWriteHandle);
        return (5);
    }

    /*
     * Get the current contents of the memory we're about to overwrite.
     * We do this since we often have to write more bytes than the user
     * requested, and we don't want to change the contents beyond what we were
     * asked to write.
     */
    std::vector<uint32_t> currentMemContents;
    currentMemContents.resize(writeLenBytes / 4);
    if (_ddrMemRead(p716xRegs, bank, writeLenBytes, currentMemContents.data(), hDev) != 0) {
        return (8);
    }
    // Copy the current memory contents into the DMA buffer
    memcpy(dmaBuf.usrBuf, currentMemContents.data(), writeLenBytes);

    // Overwrite the first part of the DMA buffer with the user's content
    memcpy(dmaBuf.usrBuf, dataBuf, dataLen);

    /* Sync Io Caches*/
    PTK716X_DMASyncIo(&dmaBuf);

    /* Interrupt & semaphore setup --------------------------------------- */

    /* enable the DMA interrupt */
    status = PTK716X_intEnable(ddrDmaWriteHandle,
                                  PTK716X_DMA_DESCRIPTOR_FINISH,
                                  NULL, (PTK716X_INT_HANDLER)ddrMemWriteIntHandler);
    if (status != PTK716X_STATUS_OK)
    {
        PTK716X_DMAFreeMem(ddrDmaWriteHandle, &dmaBuf);
        PTK716X_DMAClose(hDev, ddrDmaWriteHandle);
        return (6);
    }

    /* create a DMA Complete semaphore for this DMA channel */
    if((sem_init(&ddrMemWriteSem,0,0))<0)
    {
        PTK716X_DMAIntDisable(ddrDmaWriteHandle);
        PTK716X_DMAFreeMem(ddrDmaWriteHandle, &dmaBuf);
        PTK716X_DMAClose(hDev, ddrDmaWriteHandle);
        return (7);
    }


    /* FIFO setup -------------------------------------------------------- */

    /* set FIFO parameter table to default values */
    P716xSetFifoDefaults(&fifoParams, P716x_FIFO_TYPE_DDR_MEM);

    /* flush the FIFOs */
    P716xFlushFifo (
        (P716x_FIFO_CTRL_REG_ADDR *)&(p716xRegs->BAR2RegAddr.ddrMemWriteFifo),
        &fifoParams);

    /* enable the FIFO */
    P716x_SET_FIFO_CTRL_FIFO_ENABLE(                     \
        p716xRegs->BAR2RegAddr.ddrMemWriteFifo.FifoCtrl, \
        P716x_FIFO_ENABLE);


    /* DMA channel setup ------------------------------------------------- */

    /* Flush DMA channel buffer */
    P716xDmaFlush(&(p716xRegs->BAR0RegAddr), P716x_DMA_CHAN_7);

    /* Load parameter table with default values */
    P716xSetDmaDefaults(&dmaParams);

    /* set up channel parameters */
    P716xDmaChanSetup(&(dmaParams.dmaChan[P716x_DMA_CHAN_7]),
                      PCI716x_DMA_CMD_STAT_DMA_ENABLE,
                      PCI716x_DMA_CMD_STAT_DEMAND_MODE_DISABLE,
                      PCI716x_DMA_CMD_STAT_DATA_WIDTH_64,
                      2048,              /* Max Burst Count */
                      0);

    /* disable DAC buffering.  This parameter is not set by the above
     * function and is only required for DMA writes to the delay memory
     * when DMA Channel 6 or 7 is used.
     */
    dmaParams.dmaChan[P716x_DMA_CHAN_7].dmaDacBuffering =
        PCI716x_DMA_CMD_STAT_DAC_BUFFERING_DISABLE;

    /* setup descriptor parameters */
    P716xDmaDescptrSetup(&(dmaParams.dmaChan[P716x_DMA_CHAN_7]),
                         P716x_DMA_DESCPTR_0,
                         writeLenBytes,  /* Transfer Count bytes */
                         PCI716x_DMA_DESCPTR_XFER_CNT_INTR_DISABLE,
                         PCI716x_DMA_DESCPTR_XFER_CNT_CHAIN_END,
                         (unsigned long)dmaBuf.kernBuf);

    /* apply the parameters to the DMA registers */
    P716xDmaChanInit(&(dmaParams.dmaChan[P716x_DMA_CHAN_7]),
                     &(p716xRegs->BAR0RegAddr),
                     P716x_DMA_CHAN_7);

    /* remap DMA channel for DDR Memory writes */
    PCI716x_SET_LCL_DMA7_OUT_ADDR(             \
        p716xRegs->BAR0RegAddr.lclDmaOutRemap, \
        PCI716x_DMA7_MAP_DDR_MEM_WR_FIFO);


    /* DDR memory setup ------------------------------------------------ */

    /* load parameter tables with default values
     *
     * note: The DDR memory registers were set to default values in main()
     *       by the call to P716xResetRegs().  This routine does NOT restore
     *       the registers to default values on exit.
     */
    P716xSetDdrMemDefaults(&ddrMemParams);

    /* set DDR Memory parameters - these are set based on program mode.
     * They select DDR memory input source and output destination, enable
     * the bank, etc.  The parameters are:
     *     ddrMemCtrlRdWrFifoBankSel - bank select
     *     ddrMemCtrlBankDir         - data direction
     *     ddrMemCtrlBank0Enable     - bank 0 enable
     *     ddrMemCtrlBank1Enable     - bank 1 enable
     *     ddrMemCtrlBank2Enable     - bank 2 enable
     *     ddrMemCtrlDacSource       - DAC data source
     *     ddrMemCtrlBank0Pack       - bank bank 0 packing mode
     *     ddrMemCtrlBank1Pack       - bank bank 1 packing mode
     * Rather that setting them individually, they will be set using the
     * ReadyFlow library function, P716xSetDdrMemCtrlParams().  Bank
     * start and bank depth (in 32-bit words) are also set.
     */
    switch (bank)
    {
        case P716x_DDR_MEM_BANK0:
            P716xSetDdrMemCtrlParams(&ddrMemParams,
                    P716x_DDR_MEM_BANK_0_WRITE_MODE);
            ddrMemParams.ddrMemBank0StartAddr = 0;
            ddrMemParams.ddrMemBank0Depth = bankDepth;
        break;

        case P716x_DDR_MEM_BANK1:
            P716xSetDdrMemCtrlParams(&ddrMemParams,
                    P716x_DDR_MEM_BANK_1_WRITE_MODE);
            ddrMemParams.ddrMemBank1StartAddr = 0;
            ddrMemParams.ddrMemBank1Depth = bankDepth;
        break;

        case P716x_DDR_MEM_BANK2:
            P716xSetDdrMemCtrlParams(&ddrMemParams,
                    P716x_DDR_MEM_BANK_2_WRITE_MODE);
            ddrMemParams.ddrMemBank2StartAddr = 0;
            ddrMemParams.ddrMemBank2Depth = bankDepth;
        break;
    }

    /* apply the parameter table to the registers */
    P716xInitDdrMemRegs(&ddrMemParams, &(p716xRegs->BAR2RegAddr));

    /* FIFO enable */
    P716x_SET_FIFO_CTRL_FIFO_ENABLE(
        p716xRegs->BAR2RegAddr.ddrMemWriteFifo.FifoCtrl,
        P716x_FIFO_ENABLE);

    /* Sync Cpu Caches*/
    PTK716X_DMASyncCpu(&dmaBuf);

    /*
     * It takes two write attempts to actually get everything written.
     * Why is not clear, but even Pentek's example code ends up doing this...
     */
    for (int w = 0; w < 2; w++)
    {
        /* write to DDR memory bank */
        P716xDmaStart(&(p716xRegs->BAR0RegAddr), P716x_DMA_CHAN_7);

        /* wait for interrupt completion */
        status = sem_wait(&ddrMemWriteSem);
        if (status != 0)
        {
            sem_destroy(&ddrMemWriteSem);
            PTK716X_DMAIntDisable(ddrDmaWriteHandle);
            PTK716X_DMAFreeMem(ddrDmaWriteHandle, &dmaBuf);
            PTK716X_DMAClose(hDev, ddrDmaWriteHandle);
            return (8);
        }
    }

    /* clean up and exit ------------------------------------------------- */

    /* disable DDR memory */
    P716x_SET_DDR_MEM_MODE(p716xRegs->BAR2RegAddr.ddrMem.ddrMemCtrl,
            P716x_DDR_MEM_DISABLE_MODE);

    /* FIFO disable */
    P716x_SET_FIFO_CTRL_FIFO_ENABLE(
            p716xRegs->BAR2RegAddr.ddrMemWriteFifo.FifoCtrl,
            P716x_FIFO_DISABLE);

    /* clean up */
    sem_destroy(&ddrMemWriteSem);
    PTK716X_DMAIntDisable(ddrDmaWriteHandle);
    PTK716X_DMAFreeMem(ddrDmaWriteHandle, &dmaBuf);
    PTK716X_DMAClose(hDev, ddrDmaWriteHandle);

    // Read back and verify the written memory. Call it paranoia, but it's
    // well justified. (See the need for multiple writes above...)
    std::vector<uint32_t> readback;
    readback.resize(dataLen / 4);
    if (_ddrMemRead(p716xRegs, bank, dataLen, readback.data(), hDev) != 0) {
        return (9);
    }
    for (unsigned int i = 0; i < dataLen / 4; i++) {
        if (readback[i] != dataBuf[i]) {
            std::cerr << __PRETTY_FUNCTION__ <<
                    ": Readback mismatch at word " <<
                    i << " of " << dataLen / 4 << std::endl;
            return (9);
        }
    }

    return (0);
}

/////////////////////////////////////////////////////////////////////
double
p716x::_gauss(double mean, double stdDev) {

    // create a normally distributed random number,
    // using this nifty little algorithm.

    double x = rand()/(1.0*RAND_MAX);
    double y = rand()/(1.0*RAND_MAX);
    double u = sqrt(-2.0*log10(x))*cos(2.0*M_PI*y);

    // set the mean std deviation
    return stdDev * u + mean;
}

////////////////////////////////////////////////////////////////////////////////////////
int
p716x::_bufset(int fd, int intbufsize, int bufN) {
    /// @todo The dma buffersize should be adjusted based on
    /// a specified desired dma interrupt rate. It will be up
    /// to the user to figure out what that should be, usually
    /// in terms of the desired number of beams per interrupt.
    return 0;
}

/****************************************************************************
 Function: _ddrMemRead

 Description: reads data from the selected DDR memory bank using the
              DMA Channel 8.

 Inputs:      p716xRegs     - pointer to the 716x register addres table
              bank          - use defines:
                                  P716x_DDR_MEM_BANK0
                                  P716x_DDR_MEM_BANK1
                                  P716x_DDR_MEM_BANK2
              dataLen       - number bytes to read
              dataBuf       - pointer to the data buffer to store read data
              hDev          - 716x Device Handle

 Returns:     0 - successful
              1 - invalid bank number
              3 - bank depth extends past the end of the DDR bank
              4 - DMA channel failed to open
              5 - DMA buffer allocation failed
              6 - semaphore creation failed
              7 - semaphore wait timed out
****************************************************************************/
int p716x::_ddrMemRead(P716x_BOARD_RESOURCE *p716xRegs,
                unsigned int    bank,
                unsigned int    dataLen,
                unsigned int   *dataBuf,
                PVOID           hDev)

{
    PTK716X_DMA_HANDLE  *ddrDmaReadHandle;
    PTK716X_DMA_BUFFER   dmaBuf;
    P716x_FIFO_PARAMS    fifoParams;   /* FIFO params */
    P716x_DMA_PARAMS     dmaParams;    /* DMA params */
    P716x_DDR_MEM_PARAMS ddrMemParams; /* DDR memory params */
    unsigned int         readLen = 32 * ((dataLen + 31) / 32); /* Read in multiples of 32 bytes */
    int                  status;


    /* check input parameters -------------------------------------------- */

    /* check bank */
    if ( (bank != P716x_DDR_MEM_BANK0) &&
         (bank != P716x_DDR_MEM_BANK1) &&
         (bank != P716x_DDR_MEM_BANK2)    )
        return (1);

    /* check readLen */
    if (readLen > P716x_DDR_MEM_BANK_BYTE_SIZE)
        return (3);


    /* DMA setup --------------------------------------------------------- */

    /* open a DMA channel to read from the delay memory */
    status = PTK716X_DMAOpen(hDev, P716x_DMA_CHAN_8, &ddrDmaReadHandle);
    if (status != PTK716X_STATUS_OK)
        return (4);

    /* allocate system memory for read data buffer */
    status = PTK716X_DMAAllocMem(ddrDmaReadHandle, readLen, &dmaBuf, (BOOL)0);
    if (status != PTK716X_STATUS_OK)
    {
        PTK716X_DMAClose(hDev, ddrDmaReadHandle);
        return (5);
    }


    /* Interrupt & semaphore setup --------------------------------------- */
    /* enable the DMA interrupt */
    status = PTK716X_DMAIntEnable(ddrDmaReadHandle,
                                  PTK716X_DMA_DESCRIPTOR_FINISH,
                                  NULL, (PTK716X_INT_HANDLER)ddrMemReadIntHandler);
    if (status != PTK716X_STATUS_OK)
        {
        PTK716X_DMAFreeMem(ddrDmaReadHandle, &dmaBuf);
        PTK716X_DMAClose(hDev, ddrDmaReadHandle);
        return (6);
        }

    /* create a DMA Complete semaphore for this DMA channel */
    if((sem_init(&ddrMemReadSem,0,0))<0)
        {
        PTK716X_DMAIntDisable(ddrDmaReadHandle);
        PTK716X_DMAFreeMem(ddrDmaReadHandle, &dmaBuf);
        PTK716X_DMAClose(hDev, ddrDmaReadHandle);
        return (7);
        }


    /* FIFO setup -------------------------------------------------------- */

    /* set FIFO parameter table to default values */
    P716xSetFifoDefaults(&fifoParams, P716x_FIFO_TYPE_DDR_MEM);

    /* flush the FIFOs */
    P716xFlushFifo (
        (P716x_FIFO_CTRL_REG_ADDR *)&(p716xRegs->BAR2RegAddr.ddrMemReadFifo),
        &fifoParams);

    /* enable the FIFO */
    P716x_SET_FIFO_CTRL_FIFO_ENABLE(                    \
        p716xRegs->BAR2RegAddr.ddrMemReadFifo.FifoCtrl, \
        P716x_FIFO_ENABLE);


    /* DMA channel setup ------------------------------------------------- */

    /* Load parameter table with default values */
    P716xSetDmaDefaults(&dmaParams);

    /* set up channel parameters */
    P716xDmaChanSetup(&(dmaParams.dmaChan[P716x_DMA_CHAN_8]),
                      PCI716x_DMA_CMD_STAT_DMA_ENABLE,
                      PCI716x_DMA_CMD_STAT_DEMAND_MODE_DISABLE,
                      PCI716x_DMA_CMD_STAT_DATA_WIDTH_64,
                      512,             /* Max Burst Count */
                      0);

    /* setup descriptor parameters */
    P716xDmaDescptrSetup(&(dmaParams.dmaChan[P716x_DMA_CHAN_8]),
                         P716x_DMA_DESCPTR_0,
                         readLen,  /* Transfer Count bytes */
                         PCI716x_DMA_DESCPTR_XFER_CNT_INTR_DISABLE,
                         PCI716x_DMA_DESCPTR_XFER_CNT_CHAIN_END,
                         (unsigned long)dmaBuf.kernBuf);

    /* apply the parameters to the DMA registers */
    P716xDmaChanInit(&(dmaParams.dmaChan[P716x_DMA_CHAN_8]),
                     &(p716xRegs->BAR0RegAddr),
                     P716x_DMA_CHAN_8);

    /* Flush DMA channel buffer */
    P716xDmaFlush(&(p716xRegs->BAR0RegAddr), P716x_DMA_CHAN_8);


    /* DDR memory setup ------------------------------------------------ */

    /* load parameter tables with default values
     *
     * note: The DDR memory registers were set to default values in main()
     *       by the call to P716xResetRegs().  This routine does NOT restore
     *       the registers to default values on exit.
     */
    P716xSetDdrMemDefaults(&ddrMemParams);

    /* set DDR Memory parameters - these are set based on program mode.
     * They select DDR memory input source and output destination, enable
     * the bank, etc.  The parameters are:
     *     ddrMemCtrlRdWrFifoBankSel - bank select
     *     ddrMemCtrlBankDir         - data direction
     *     ddrMemCtrlBank0Enable     - bank 0 enable
     *     ddrMemCtrlBank1Enable     - bank 1 enable
     *     ddrMemCtrlBank2Enable     - bank 2 enable
     *     ddrMemCtrlDacSource       - DAC data source
     *     ddrMemCtrlBank0Pack       - bank bank 0 packing mode
     *     ddrMemCtrlBank1Pack       - bank bank 1 packing mode
     * Rather that setting them individually, they will be set using the
     * ReadyFlow library function, P716xSetDdrMemCtrlParams().  Bank
     * start and bank depth (in 32-bit words) are also set.
     */
    switch (bank)
    {
        case P716x_DDR_MEM_BANK0:
            P716xSetDdrMemCtrlParams(&ddrMemParams,
                                      P716x_DDR_MEM_BANK_0_READ_MODE);
            ddrMemParams.ddrMemBank0StartAddr = 0;
            ddrMemParams.ddrMemBank0Depth = (readLen/4);
        break;

        case P716x_DDR_MEM_BANK1:
            P716xSetDdrMemCtrlParams(&ddrMemParams,
                                      P716x_DDR_MEM_BANK_1_READ_MODE);
            ddrMemParams.ddrMemBank1StartAddr = 0;
            ddrMemParams.ddrMemBank1Depth = (readLen/4);
        break;

        case P716x_DDR_MEM_BANK2:
            P716xSetDdrMemCtrlParams(&ddrMemParams,
                                      P716x_DDR_MEM_BANK_2_READ_MODE);
            ddrMemParams.ddrMemBank2StartAddr = 0;
            ddrMemParams.ddrMemBank2Depth = (readLen/4);
        break;
    }

    /* apply the parameter table to the registers */
    P716xInitDdrMemRegs(&ddrMemParams, &(p716xRegs->BAR2RegAddr));

    /* start the transfer, wait for completion --------------------------- */

    /* read from DDR Memory Bank */
    P716xDmaStart(&(p716xRegs->BAR0RegAddr), P716x_DMA_CHAN_8);

    /* wait for interrupt completion */
    status = sem_wait(&ddrMemReadSem);
    if (status != 0)
    {
        sem_destroy(&ddrMemReadSem);
        PTK716X_DMAIntDisable(ddrDmaReadHandle);
        PTK716X_DMAFreeMem(ddrDmaReadHandle, &dmaBuf);
        PTK716X_DMAClose(hDev, ddrDmaReadHandle);
        return (8);
    }

    /* Sync Io Caches*/
    PTK716X_DMASyncIo(&dmaBuf);

    /* copy desired bytes from DMA buffer to data buffer */
    memcpy(dataBuf, dmaBuf.usrBuf, dataLen);


    /* clean up and exit ------------------------------------------------- */

    /* disable DDR memory */
    P716x_SET_DDR_MEM_MODE(p716xRegs->BAR2RegAddr.ddrMem.ddrMemCtrl,   \
                           P716x_DDR_MEM_DISABLE_MODE);

    /* FIFO disable */
    P716x_SET_FIFO_CTRL_FIFO_ENABLE(                                   \
        p716xRegs->BAR2RegAddr.ddrMemReadFifo.FifoCtrl,                \
        P716x_FIFO_DISABLE);

    /* clean up */
    sem_destroy(&ddrMemReadSem);
    PTK716X_intDisable(ddrDmaReadHandle);
    PTK716X_DMAFreeMem(ddrDmaReadHandle, &dmaBuf);
    PTK716X_DMAClose(hDev, ddrDmaReadHandle);

    return (0);
}

/////////////////////////////////////////////////////////////////////
uint32_t p716x::nextSimPulseNum(int chan) {

	///@todo This class needs to be updated to cause the pulse number wrap
	/// around to occur at the right number, depending on whether we are in
	/// pulse tagger or coherent integrator mode.

	boost::unique_lock<boost::mutex> lock(_simPulseNumMutex);
    _waitingDownconverters++;
    if (_waitingDownconverters < (_downconverters.size())) {
    	//DLOG << "chan " << chan << " waiting for pulse number";
    	_simPulseNumCondition.wait(lock);
    } else {
    	simWait();
    	_waitingDownconverters = 0;
    	_simPulseNumCondition.notify_all();
    }
    // DLOG << "chan " << chan << " gets pulse number "
    //      << _downconverters[chan]._pulseNum + 1;
    return(_downconverters[chan]._pulseNum++);

}
//////////////////////////////////////////////////////////////////////////////////
void
p716x::simWait() {
    // because the usleep overhead is large, sleep every 100 calls
   	if (_simPauseMS > 0) {
   	    if (!(_simWaitCounter++ % 100)) {
        	usleep((int)(_simPauseMS*1000));
    	}
    }
}
//////////////////////////////////////////////////////////////////////////////////
int
p716x::circuitBoardTemp() const {
    // Return a fixed value if simulating
    if (_simulate) {
        return(35);
    }

    LM83_VALUES     tempMonValues;
    /* read the LM83 temperature registers */
    Twsi_LM83GetValues((unsigned long)(_boardResource.BAR2RegAddr.twsiPort),
            P716x_TWSI_ADDR_LM83, &tempMonValues);
    // The D3 temperature sensor is on the PCB.
    int temp = tempMonValues.D3Temp;
    // Pentek's library doesn't do the right thing unpacking temperatures
    // below 0. Correct that here.
    if (temp > 127)
    	temp -= 256;
    return(temp);
}
//////////////////////////////////////////////////////////////////////////////////
int
p716x::fpgaTemp() const {
    // Return a fixed value if simulating
    if (_simulate) {
        return(50);
    }

    LM83_VALUES     tempMonValues;
    /* read the LM83 temperature registers */
    Twsi_LM83GetValues((unsigned long)(_boardResource.BAR2RegAddr.twsiPort),
            P716x_TWSI_ADDR_LM83, &tempMonValues);
    // The D2 temperature sensor is on the signal processing FPGA.
    int temp = tempMonValues.D2Temp;
    // Pentek's library doesn't do the right thing unpacking temperatures
    // below 0. Correct that here.
    if (temp > 127)
    	temp -= 256;
    return(temp);
}
