#include "p7142.h"
#include "p7142Dn.h"
#include "p7142Up.h"
#include <fcntl.h>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <csignal>

#include <logx/Logging.h>
LOGGING("p7142");

using namespace Pentek;

////////////////////////////////////////////////////////////////////////////////////////

/// Semaphore used by the _ddrMemWrite() routine to signal that an interrupt was
/// received from a dma transfer
sem_t ddrMemWriteSem;

/// Interrupt handler used by the ddrMemWrite() DMA transfers
void ddrMemWriteIntHandler(PVOID               hDev,
                        int                lintSource,
                        PVOID               pData,
                        PTK714X_INT_RESULT *pIntResult)
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
                       PTK714X_INT_RESULT *pIntResult)
{
    sem_post(&ddrMemReadSem);
}

/// Static member to keep track of the "PCI slot" of the last instantiated p7142.
/// This is needed for calls to ReadyFlow's PTK714X_DeviceFindAndOpen() 
/// function. To open 7142 cards in sequence, we start with _Next7142Slot = -2.
/// See the ReadyFlow documentation for PTK714X_DeviceFindAndOpen().
DWORD p7142::_Next7142Slot = -2;

/// Static member to keep track of how many 7142 cards we have open (i.e., how
/// many instances of this class are there so far).
uint16_t p7142::_NumOpenCards = 0;

////////////////////////////////////////////////////////////////////////////////////////
p7142::p7142(bool simulate, double simPauseMS, bool useFirstCard):
_cardIndex(_NumOpenCards),
_simulate(simulate),
_p7142Mutex(),
_isReady(false),
_upconverter(0),
_simPulseNum(0),
_waitingDownconverters(0),
_simWaitCounter(0),
_simPauseMS(simPauseMS)
{
	boost::recursive_mutex::scoped_lock guard(_p7142Mutex);

	// If we're simulating, things are simple...
    if (_simulate) {
        _isReady = true;
    } else {
    	// initialize ReadyFlow
    	if (useFirstCard) {
    		// we have been explicitly told to find the first card.
    		_Next7142Slot = -2;
    	}
    	_isReady = _initReadyFlow();
    }
    // If we were successful, increment the open card count
    if (_isReady)
        _NumOpenCards++;

    return;
}

////////////////////////////////////////////////////////////////////////////////
p7142::~p7142() {
    if (_simulate) {
        return;
    }

    boost::recursive_mutex::scoped_lock guard(_p7142Mutex);

    // destroy the down converters
    for (std::map<int, DownconverterInfo>::iterator i = _downconverters.begin();
    	i != _downconverters.end(); i++) {
        delete i->second._dn;
    }
    // empty the list of active down converters
    _downconverters.erase(_downconverters.begin(), _downconverters.end());
    
    // delete the upconverter, if any
    delete _upconverter;

    /* cleanup for exit */
    PTK714X_DeviceClose(_deviceHandle);
    _NumOpenCards--;
    
    // If there are no instances left, set to open the first Pentek card by
    // default on the next instantiation, and close up the ReadyFlow library.
    if (_NumOpenCards == 0) {
        _Next7142Slot = -2; // use the first card in the system next time
        PTK714X_LibUninit();
        DLOG << "ReadyFlow closed";
    }
}

////////////////////////////////////////////////////////////////////////////////
p7142Dn*
p7142::addDownconverter(int chanId, uint32_t dmaDescSize, int bypassdivrate,
        int simWavelength, bool sim4bytes) {
    boost::recursive_mutex::scoped_lock guard(_p7142Mutex);

    // Just construct a new downconverter and put it in our list.
    p7142Dn* downconverter = new p7142Dn(
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
p7142Up*
p7142::addUpconverter(
		double sampleClockHz,
        double ncoFreqHz,
        char mode) {
    boost::recursive_mutex::scoped_lock guard(_p7142Mutex);
    // Just construct a new upconverter and put it in our list.
    p7142Up* upconverter = new p7142Up(
    		this,
    		sampleClockHz,
    		ncoFreqHz,
    		mode);
    _addUpconverter(upconverter);
    return(upconverter);
}

////////////////////////////////////////////////////////////////////////////////////////
int p7142::memWrite(int bank, int32_t* buf, int bytes) {

    int retval = bytes;

    int status = _ddrMemWrite (
            &_p7142Regs,
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
int p7142::memRead(int bank, int32_t* buf, int bytes) {

    int retval = bytes;

    int status = _ddrMemRead (
            &_p7142Regs,
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
p7142::_initReadyFlow() {

    _deviceHandle = NULL;

    /* initialize the PTK714X library */
    DWORD dwStatus = PTK714X_LibInit();
    if (dwStatus != PTK714X_STATUS_OK)
    {
      ELOG << "Failed to initialize the PTK714X library.";
      ELOG << "Make sure that windrvr6 is loaded (did you do a kernel upgrade recently?).";
      ELOG << "Also make sure you have read permission on /dev/windrvr6.";
      return false;
    }

    /* Find and open the next PTK714X device */
    _deviceHandle = PTK714X_DeviceFindAndOpen(&_Next7142Slot, &_BAR0Base, &_BAR2Base);
    if (_deviceHandle == NULL)
    {
      ELOG << "Pentek 7142 device not found when opening card " << 
        _NumOpenCards;
      return false;
    }

    /* Initialize 7142 register address tables */
    P7142InitRegAddr (_BAR0Base, _BAR2Base, &_p7142Regs);

    // Reset the board so we start in pristine condition
    PCI7142_SET_BD_CHAN_RST_BOARD_RESET(_p7142Regs.BAR0RegAddr.bdChanReset,
            PCI7142_BD_CHAN_RST_BOARD_RESET);
    usleep(1000);
    PCI7142_SET_BD_CHAN_RST_BOARD_RESET(_p7142Regs.BAR0RegAddr.bdChanReset,
            PCI7142_BD_CHAN_RST_BOARD_RUN);
    usleep(1000);
    
    /* check if module is a 7142 */
    P7142_GET_MODULE_ID(_p7142Regs.BAR2RegAddr.idReadout, _moduleId);
    if (_moduleId != P7142_MODULE_ID)
    {
      ELOG << "Pentek card " << _NumOpenCards + 1 << " is not a 7142!";
      ELOG << "Expected 0x" << std::hex << P7142_MODULE_ID << 
        ", and got 0x" << _moduleId << std::dec;
      return false;
    }

    DLOG << "Pentek 7142 device";
    DLOG << std::hex << " BAR0: 0x" << (void *)_BAR0Base;
    DLOG << std::hex << " BAR2: 0x" << (void *)_BAR2Base;
    DLOG << std::dec;


    /// @todo Although we follow the normal ReadyFlow protocol
    /// for configuring the DAC (P7142SetDac5687Defaults()
    /// followed by P7142InitDac5687Regs()),
    /// the DAC is completely reconfigured in p7142Up().
    /// We need to modify P7142SetDac5687Defaults() and
    /// P7142InitDac5687Regs() to perform the correct configuration,
    /// so that it can be pulled out of p7142Up().

    // Reset board registers to default values
    P7142ResetRegs (&_p7142Regs);

    /* Load parameter tables with default values */
    P7142SetPciDefaults    (&_p7142PciParams);
    P7142SetDmaDefaults    (&_p7142DmaParams);
    P7142SetBoardDefaults  (&_p7142BoardParams);
    P7142SetDdrMemDefaults (&_p7142MemParams);
    P7142SetInputDefaults  (&_p7142InParams);
    P7142SetOutputDefaults (&_p7142OutParams);
    P7142SetDac5687Defaults(&_p7142Dac5686Params);

    // Apply our adjustments
    _configBoardParameters();
    _configInParameters();
    _configOutParameters();

    /* Write parameter table values to the 7142 registers */
    P7142InitPciRegs    (&_p7142PciParams,   &(_p7142Regs.BAR0RegAddr));
    P7142InitDmaRegs    (&_p7142DmaParams,   &(_p7142Regs.BAR0RegAddr));
    P7142InitBoardRegs  (&_p7142BoardParams, &(_p7142Regs.BAR2RegAddr));
    P7142InitDdrMemRegs (&_p7142MemParams,   &(_p7142Regs.BAR2RegAddr));
    P7142InitInputRegs  (&_p7142InParams,    &(_p7142Regs.BAR2RegAddr));
    P7142InitOutputRegs (&_p7142OutParams,   &(_p7142Regs.BAR2RegAddr));
    P7142InitDac5687Regs(&_p7142OutParams,   &_p7142Dac5686Params,   &(_p7142Regs.BAR2RegAddr));

    // Determine the gate generator register pointer
    if (_p7142InParams.inputSyncBusSel == P7142_SYNC_BUS_SEL_A)
        _gateGenReg = (volatile unsigned int *)(_p7142Regs.BAR2RegAddr.gateAGen);
    else
        _gateGenReg = (volatile unsigned int *)(_p7142Regs.BAR2RegAddr.gateBGen);

    // Disable the gate generator so that the fifos don't run asynchronously,
    disableGateGen();

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////
void p7142::_configBoardParameters() {

    // Board customization

    _p7142BoardParams.busAMaster      = P7142_MSTR_CTRL_MASTER;
    _p7142BoardParams.busATermination = P7142_MSTR_CTRL_TERMINATED;

    _p7142BoardParams.busASelectClock = P7142_MSTR_CTRL_SEL_CLK_EXT_CLK;
    _p7142BoardParams.busAClockSource = P7142_MSTR_CTRL_CLK_SRC_SEL_CLK;

    _p7142BoardParams.busASelectSync  = P7142_MSTR_CTRL_SEL_SYNC_REGISTER;
    _p7142BoardParams.busASyncSource  = P7142_MSTR_CTRL_SYNC_SRC_SEL_SYNC;

    _p7142BoardParams.busASelectGate  = P7142_MSTR_CTRL_SEL_GATE_REGISTER;
    _p7142BoardParams.busAGateSource  = P7142_MSTR_CTRL_GATE_SRC_SEL_GATE;

    /* Bus B parameters */

    _p7142BoardParams.busBMaster      = P7142_MSTR_CTRL_MASTER;
    _p7142BoardParams.busBTermination = P7142_MSTR_CTRL_TERMINATED;

    _p7142BoardParams.busBSelectClock = P7142_MSTR_CTRL_SEL_CLK_EXT_CLK;
    _p7142BoardParams.busBClockSource = P7142_MSTR_CTRL_CLK_SRC_SEL_CLK;

    _p7142BoardParams.busBSelectSync  = P7142_MSTR_CTRL_SEL_SYNC_REGISTER;
    _p7142BoardParams.busBSyncSource  = P7142_MSTR_CTRL_SYNC_SRC_SEL_SYNC;

    _p7142BoardParams.busBSelectGate  = P7142_MSTR_CTRL_SEL_GATE_REGISTER;
    _p7142BoardParams.busBGateSource  = P7142_MSTR_CTRL_GATE_SRC_SEL_GATE;

    _p7142BoardParams.endianness = P7142_MISC_CTRL_ENDIANNESS_LE;

}

////////////////////////////////////////////////////////////////////////////////////////
void p7142::_configInParameters() {

    // Sync Bus select
    _p7142InParams.inputSyncBusSel = P7142_SYNC_BUS_SEL_A;

    // set the down conversion FIFO parameters

    for (int adchan = 0; adchan < 4; adchan++) {
        // select data packing mode.  It can be unpacked or time-packed.
        // The program define is located at the top of the program.
        //
        _p7142InParams.adcFifo[adchan].fifoPackMode = P7142_FIFO_ADC_PACK_MODE_TIME_PACK;

        // set the FIFO decimation.  This allows the input data rate to the
        // FIFO to be reduced.  It can be a value from 0 to 0xFFF.  Actual
        // decimation is this value plus one.

        // set to decimation by 1 here, but will almost always be modified by the user
        // to an appropriate value.
        _p7142InParams.adcFifoDecimation[adchan] = 0;

        // The FIFO Almost Full and Almost Empty levels are set to default
        // values for all programs.  The values shown here are the default
        // values and and are provided to show usage.  Their values must be
        // chosen to work with the DMA channel maximum burst count value.
        _p7142InParams.adcFifo[adchan].fifoAlmostEmptyLevel = 512;
        _p7142InParams.adcFifo[adchan].fifoAlmostFullLevel  = 544;

        // Enable gate control
        _p7142InParams.adcFifo[adchan].fifoGateControl = P7142_FIFO_GATE_ENABLE;
        _p7142InParams.adcFifo[adchan].fifoGateSelect = P7142_FIFO_GATE_SELECT_A;
        _p7142InParams.adcFifo[adchan].fifoGateTrigSelect = P7142_FIFO_GATE_OPERATION;

    }

}

////////////////////////////////////////////////////////////////////////////////////////
void p7142::_configOutParameters() {

    // Customize the up conversion path

    /// @todo Is the following correct for PLL usage? What do they mean by "bypass"?
    _p7142OutParams.dacClkSel = P7142_DAC_CTRL_STAT_DAC_CLK_BYPASS;
    _p7142OutParams.dacPllVdd = P7142_DAC_CTRL_STAT_PLL_VDD_ENABLE;
    _p7142OutParams.outputSyncBusSel = P7142_SYNC_BUS_SEL_A;
    // Following pack mode is used becasue we are putting I and Q into a 32 bit memory word.
    _p7142OutParams.dacFifo.fifoPackMode = P7142_FIFO_DAC_PACK_MODE_UNPACK;
    _p7142OutParams.dacFifo.fifoWordSwap = P7142_FIFO_CTRL_WORD_SWAP_DISABLE;

    // The FIFO Almost Full and Almost Empty levels are set to default
    // values for all programs.  The values shown here are the default
    // values for DAC FIFOs and are provided to show usage.   Their values
    // must be chosen to work with the DMA channel maximum burst count
    // value.
    _p7142OutParams.dacFifo.fifoAlmostEmptyLevel = 6144;
    _p7142OutParams.dacFifo.fifoAlmostFullLevel  = 6176;
}

////////////////////////////////////////////////////////////////////////////////////////
void
p7142::enableGateGen() {

	P7142_SET_GATE_GEN(_gateGenReg, P7142_GATE_GEN_ENABLE);
    DLOG << "GateGen enabled";
}

////////////////////////////////////////////////////////////////////////////////////////
void
p7142::disableGateGen() {

	P7142_SET_GATE_GEN(_gateGenReg, P7142_GATE_GEN_DISABLE);
    DLOG << "GateGen disabled";
}

////////////////////////////////////////////////////////////////////////////////
void
p7142::_addDownconverter(p7142Dn * downconverter) {
    boost::recursive_mutex::scoped_lock guard(_p7142Mutex);
    
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
p7142::_addUpconverter(p7142Up * upconverter) {
    boost::recursive_mutex::scoped_lock guard(_p7142Mutex);

    if (_upconverter) {
        std::cerr << "Existing upconverter is being replaced"  << std::endl;
        delete _upconverter;
    }
    _upconverter = upconverter;
}

////////////////////////////////////////////////////////////////////////////////////////
void
p7142::_resetDCM() {
    boost::recursive_mutex::scoped_lock guard(_p7142Mutex);

    if (isSimulating())
        return;

    // cycle the digital clock manager
    // hold the dcm in reset for a short period
    P7142_SET_DCM_CTRL_DCM_RST(_p7142Regs.BAR2RegAddr.dcmControl, P7142_DCM_CTRL_DCM_RST_RESET);
    usleep(1000);
    // take the dcm out of reset
    P7142_SET_DCM_CTRL_DCM_RST(_p7142Regs.BAR2RegAddr.dcmControl, P7142_DCM_CTRL_DCM_RST_RUN);
    usleep(1000);

    DLOG << "DCM has been cycled.";

    return;

}

////////////////////////////////////////////////////////////////////////////////////////
/****************************************************************************
 Function: _ddrMemWrite

 Description: writes data to the selected DDR memory bank.  Uses DMA
              Channel 7.

 Inputs:      p7142Regs     - pointer to the 7142 register addres table
              bank          - use defines:
                                  P7142_DDR_MEM_BANK0
                                  P7142_DDR_MEM_BANK1
                                  P7142_DDR_MEM_BANK2
              dataLen       - number bytes to write
              dataBuf       - pointer to the data buffer containing the data
              hDev          - 7142 Device Handle

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
int p7142::_ddrMemWrite (P7142_REG_ADDR* p7142Regs,
                 unsigned int    bank,
                 unsigned int    dataLen,
                 uint32_t*       dataBuf,
                 PVOID           hDev)
{
    PTK714X_DMA_HANDLE  *ddrDmaWriteHandle;
    PTK714X_DMA_BUFFER   dmaBuf;
    P7142_FIFO_PARAMS    fifoParams;   /* FIFO params */
    P7142_DMA_PARAMS     dmaParams;    /* DMA params */
    P7142_DDR_MEM_PARAMS ddrMemParams; /* DDR memory params */
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
    if ( (bank != P7142_DDR_MEM_BANK0) &&
         (bank != P7142_DDR_MEM_BANK1) &&
         (bank != P7142_DDR_MEM_BANK2)    )
        return (1);

    /* check dataLen */
    if (writeLenBytes > P7142_DDR_MEM_BANK_BYTE_SIZE)
        return (3);


    /* DMA setup --------------------------------------------------------- */

    /* open a DMA channel to write to the delay memory */
    status = PTK714X_DMAOpen(hDev, P7142_DMA_CHAN_7, &ddrDmaWriteHandle);
    if (status != PTK714X_STATUS_OK)
        return (4);

    /* allocate system memory for write data buffer */
    status = PTK714X_DMAAllocMem(ddrDmaWriteHandle, writeLenBytes, &dmaBuf, (BOOL)0);
    if (status != PTK714X_STATUS_OK)
    {
        PTK714X_DMAClose(hDev, ddrDmaWriteHandle);
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
    if (_ddrMemRead(p7142Regs, bank, writeLenBytes, currentMemContents.data(), hDev) != 0) {
        return (8);
    }
    // Copy the current memory contents into the DMA buffer
    memcpy(dmaBuf.usrBuf, currentMemContents.data(), writeLenBytes);

    // Overwrite the first part of the DMA buffer with the user's content
    memcpy(dmaBuf.usrBuf, dataBuf, dataLen);

    /* Sync Io Caches*/
    PTK714X_DMASyncIo(&dmaBuf);

    /* Interrupt & semaphore setup --------------------------------------- */

    /* enable the DMA interrupt */
    status = PTK714X_DMAIntEnable(ddrDmaWriteHandle,
                                  PTK714X_DMA_DESCRIPTOR_FINISH,
                                  NULL, (PTK714X_INT_HANDLER)ddrMemWriteIntHandler);
    if (status != PTK714X_STATUS_OK)
    {
        PTK714X_DMAFreeMem(ddrDmaWriteHandle, &dmaBuf);
        PTK714X_DMAClose(hDev, ddrDmaWriteHandle);
        return (6);
    }

    /* create a DMA Complete semaphore for this DMA channel */
    if((sem_init(&ddrMemWriteSem,0,0))<0)
    {
        PTK714X_DMAIntDisable(ddrDmaWriteHandle);
        PTK714X_DMAFreeMem(ddrDmaWriteHandle, &dmaBuf);
        PTK714X_DMAClose(hDev, ddrDmaWriteHandle);
        return (7);
    }


    /* FIFO setup -------------------------------------------------------- */

    /* set FIFO parameter table to default values */
    P7142SetFifoDefaults(&fifoParams, P7142_FIFO_TYPE_DDR_MEM);

    /* flush the FIFOs */
    P7142FlushFifo (
        (P7142_FIFO_CTRL_REG_ADDR *)&(p7142Regs->BAR2RegAddr.ddrMemWriteFifo),
        &fifoParams);

    /* enable the FIFO */
    P7142_SET_FIFO_CTRL_FIFO_ENABLE(                     \
        p7142Regs->BAR2RegAddr.ddrMemWriteFifo.FifoCtrl, \
        P7142_FIFO_ENABLE);


    /* DMA channel setup ------------------------------------------------- */

    /* Flush DMA channel buffer */
    P7142DmaFlush(&(p7142Regs->BAR0RegAddr), P7142_DMA_CHAN_7);

    /* Load parameter table with default values */
    P7142SetDmaDefaults (&dmaParams);

    /* set up channel parameters */
    P7142DmaChanSetup(&(dmaParams.dmaChan[P7142_DMA_CHAN_7]),
                      PCI7142_DMA_CMD_STAT_DMA_ENABLE,
                      PCI7142_DMA_CMD_STAT_DEMAND_MODE_DISABLE,
                      PCI7142_DMA_CMD_STAT_DATA_WIDTH_64,
                      2048,              /* Max Burst Count */
                      0);

    /* disable DAC buffering.  This parameter is not set by the above
     * function and is only required for DMA writes to the delay memory
     * when DMA Channel 6 or 7 is used.
     */
    dmaParams.dmaChan[P7142_DMA_CHAN_7].dmaDacBuffering =
        PCI7142_DMA_CMD_STAT_DAC_BUFFERING_DISABLE;

    /* setup descriptor parameters */
    P7142DmaDescptrSetup(&(dmaParams.dmaChan[P7142_DMA_CHAN_7]),
                         P7142_DMA_DESCPTR_0,
                         writeLenBytes,  /* Transfer Count bytes */
                         PCI7142_DMA_DESCPTR_XFER_CNT_INTR_DISABLE,
                         PCI7142_DMA_DESCPTR_XFER_CNT_CHAIN_END,
                         (unsigned long)dmaBuf.kernBuf);

    /* apply the parameters to the DMA registers */
    P7142DmaChanInit(&(dmaParams.dmaChan[P7142_DMA_CHAN_7]),
                     &(p7142Regs->BAR0RegAddr),
                     P7142_DMA_CHAN_7);

    /* remap DMA channel for DDR Memory writes */
    PCI7142_SET_LCL_DMA7_OUT_ADDR(             \
        p7142Regs->BAR0RegAddr.lclDmaOutRemap, \
        PCI7142_DMA7_MAP_DDR_MEM_WR_FIFO);


    /* DDR memory setup ------------------------------------------------ */

    /* load parameter tables with default values
     *
     * note: The DDR memory registers were set to default values in main()
     *       by the call to P7142ResetRegs().  This routine does NOT restore
     *       the registers to default values on exit.
     */
    P7142SetDdrMemDefaults(&ddrMemParams);

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
     * ReadyFlow library function, P7142SetDdrMemCtrlParams().  Bank
     * start and bank depth (in 32-bit words) are also set.
     */
    switch (bank)
    {
        case P7142_DDR_MEM_BANK0:
            P7142SetDdrMemCtrlParams(&ddrMemParams,
                    P7142_DDR_MEM_BANK_0_WRITE_MODE);
            ddrMemParams.ddrMemBank0StartAddr = 0;
            ddrMemParams.ddrMemBank0Depth = bankDepth;
        break;

        case P7142_DDR_MEM_BANK1:
            P7142SetDdrMemCtrlParams(&ddrMemParams,
                    P7142_DDR_MEM_BANK_1_WRITE_MODE);
            ddrMemParams.ddrMemBank1StartAddr = 0;
            ddrMemParams.ddrMemBank1Depth = bankDepth;
        break;

        case P7142_DDR_MEM_BANK2:
            P7142SetDdrMemCtrlParams(&ddrMemParams,
                    P7142_DDR_MEM_BANK_2_WRITE_MODE);
            ddrMemParams.ddrMemBank2StartAddr = 0;
            ddrMemParams.ddrMemBank2Depth = bankDepth;
        break;
    }

    /* apply the parameter table to the registers */
    P7142InitDdrMemRegs (&ddrMemParams, &(p7142Regs->BAR2RegAddr));

    /* FIFO enable */
    P7142_SET_FIFO_CTRL_FIFO_ENABLE(
        p7142Regs->BAR2RegAddr.ddrMemWriteFifo.FifoCtrl,
        P7142_FIFO_ENABLE);

    /* Sync Cpu Caches*/
    PTK714X_DMASyncCpu(&dmaBuf);

    /*
     * It takes two write attempts to actually get everything written.
     * Why is not clear, but even Pentek's example code ends up doing this...
     */
    for (int w = 0; w < 2; w++)
    {
        /* write to DDR memory bank */
        P7142DmaStart(&(p7142Regs->BAR0RegAddr), P7142_DMA_CHAN_7);

        /* wait for interrupt completion */
        status = sem_wait(&ddrMemWriteSem);
        if (status != 0)
        {
            sem_destroy(&ddrMemWriteSem);
            PTK714X_DMAIntDisable(ddrDmaWriteHandle);
            PTK714X_DMAFreeMem(ddrDmaWriteHandle, &dmaBuf);
            PTK714X_DMAClose(hDev, ddrDmaWriteHandle);
            return (8);
        }
    }

    /* clean up and exit ------------------------------------------------- */

    /* disable DDR memory */
    P7142_SET_DDR_MEM_MODE(p7142Regs->BAR2RegAddr.ddrMem.ddrMemCtrl,
            P7142_DDR_MEM_DISABLE_MODE);

    /* FIFO disable */
    P7142_SET_FIFO_CTRL_FIFO_ENABLE(
            p7142Regs->BAR2RegAddr.ddrMemWriteFifo.FifoCtrl,
            P7142_FIFO_DISABLE);

    /* clean up */
    sem_destroy(&ddrMemWriteSem);
    PTK714X_DMAIntDisable(ddrDmaWriteHandle);
    PTK714X_DMAFreeMem(ddrDmaWriteHandle, &dmaBuf);
    PTK714X_DMAClose(hDev, ddrDmaWriteHandle);

    // Read back and verify the written memory. Call it paranoia, but it's
    // well justified. (See the need for multiple writes above...)
    std::vector<uint32_t> readback;
    readback.resize(dataLen / 4);
    if (_ddrMemRead(p7142Regs, bank, dataLen, readback.data(), hDev) != 0) {
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
p7142::_gauss(double mean, double stdDev) {

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
p7142::_bufset(int fd, int intbufsize, int bufN) {
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

 Inputs:      p7142Regs     - pointer to the 7142 register addres table
              bank          - use defines:
                                  P7142_DDR_MEM_BANK0
                                  P7142_DDR_MEM_BANK1
                                  P7142_DDR_MEM_BANK2
              dataLen       - number bytes to read
              dataBuf       - pointer to the data buffer to store read data
              hDev          - 7142 Device Handle

 Returns:     0 - successful
              1 - invalid bank number
              3 - bank depth extends past the end of the DDR bank
              4 - DMA channel failed to open
              5 - DMA buffer allocation failed
              6 - semaphore creation failed
              7 - semaphore wait timed out
****************************************************************************/
int p7142::_ddrMemRead (P7142_REG_ADDR *p7142Regs,
                unsigned int    bank,
                unsigned int    dataLen,
                unsigned int   *dataBuf,
                PVOID           hDev)

{
    PTK714X_DMA_HANDLE  *ddrDmaReadHandle;
    PTK714X_DMA_BUFFER   dmaBuf;
    P7142_FIFO_PARAMS    fifoParams;   /* FIFO params */
    P7142_DMA_PARAMS     dmaParams;    /* DMA params */
    P7142_DDR_MEM_PARAMS ddrMemParams; /* DDR memory params */
    unsigned int         readLen = 32 * ((dataLen + 31) / 32); /* Read in multiples of 32 bytes */
    int                  status;


    /* check input parameters -------------------------------------------- */

    /* check bank */
    if ( (bank != P7142_DDR_MEM_BANK0) &&
         (bank != P7142_DDR_MEM_BANK1) &&
         (bank != P7142_DDR_MEM_BANK2)    )
        return (1);

    /* check readLen */
    if (readLen > P7142_DDR_MEM_BANK_BYTE_SIZE)
        return (3);


    /* DMA setup --------------------------------------------------------- */

    /* open a DMA channel to read from the delay memory */
    status = PTK714X_DMAOpen(hDev, P7142_DMA_CHAN_8, &ddrDmaReadHandle);
    if (status != PTK714X_STATUS_OK)
        return (4);

    /* allocate system memory for read data buffer */
    status = PTK714X_DMAAllocMem(ddrDmaReadHandle, readLen, &dmaBuf, (BOOL)0);
    if (status != PTK714X_STATUS_OK)
    {
        PTK714X_DMAClose(hDev, ddrDmaReadHandle);
        return (5);
    }


    /* Interrupt & semaphore setup --------------------------------------- */
    /* enable the DMA interrupt */
    status = PTK714X_DMAIntEnable(ddrDmaReadHandle,
                                  PTK714X_DMA_DESCRIPTOR_FINISH,
                                  NULL, (PTK714X_INT_HANDLER)ddrMemReadIntHandler);
    if (status != PTK714X_STATUS_OK)
        {
        PTK714X_DMAFreeMem(ddrDmaReadHandle, &dmaBuf);
        PTK714X_DMAClose(hDev, ddrDmaReadHandle);
        return (6);
        }

    /* create a DMA Complete semaphore for this DMA channel */
    if((sem_init(&ddrMemReadSem,0,0))<0)
        {
        PTK714X_DMAIntDisable(ddrDmaReadHandle);
        PTK714X_DMAFreeMem(ddrDmaReadHandle, &dmaBuf);
        PTK714X_DMAClose(hDev, ddrDmaReadHandle);
        return (7);
        }


    /* FIFO setup -------------------------------------------------------- */

    /* set FIFO parameter table to default values */
    P7142SetFifoDefaults(&fifoParams, P7142_FIFO_TYPE_DDR_MEM);

    /* flush the FIFOs */
    P7142FlushFifo (
        (P7142_FIFO_CTRL_REG_ADDR *)&(p7142Regs->BAR2RegAddr.ddrMemReadFifo),
        &fifoParams);

    /* enable the FIFO */
    P7142_SET_FIFO_CTRL_FIFO_ENABLE(                    \
        p7142Regs->BAR2RegAddr.ddrMemReadFifo.FifoCtrl, \
        P7142_FIFO_ENABLE);


    /* DMA channel setup ------------------------------------------------- */

    /* Load parameter table with default values */
    P7142SetDmaDefaults (&dmaParams);

    /* set up channel parameters */
    P7142DmaChanSetup(&(dmaParams.dmaChan[P7142_DMA_CHAN_8]),
                      PCI7142_DMA_CMD_STAT_DMA_ENABLE,
                      PCI7142_DMA_CMD_STAT_DEMAND_MODE_DISABLE,
                      PCI7142_DMA_CMD_STAT_DATA_WIDTH_64,
                      512,             /* Max Burst Count */
                      0);

    /* setup descriptor parameters */
    P7142DmaDescptrSetup(&(dmaParams.dmaChan[P7142_DMA_CHAN_8]),
                         P7142_DMA_DESCPTR_0,
                         readLen,  /* Transfer Count bytes */
                         PCI7142_DMA_DESCPTR_XFER_CNT_INTR_DISABLE,
                         PCI7142_DMA_DESCPTR_XFER_CNT_CHAIN_END,
                         (unsigned long)dmaBuf.kernBuf);

    /* apply the parameters to the DMA registers */
    P7142DmaChanInit(&(dmaParams.dmaChan[P7142_DMA_CHAN_8]),
                     &(p7142Regs->BAR0RegAddr),
                     P7142_DMA_CHAN_8);

    /* Flush DMA channel buffer */
    P7142DmaFlush(&(p7142Regs->BAR0RegAddr), P7142_DMA_CHAN_8);


    /* DDR memory setup ------------------------------------------------ */

    /* load parameter tables with default values
     *
     * note: The DDR memory registers were set to default values in main()
     *       by the call to P7142ResetRegs().  This routine does NOT restore
     *       the registers to default values on exit.
     */
    P7142SetDdrMemDefaults(&ddrMemParams);

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
     * ReadyFlow library function, P7142SetDdrMemCtrlParams().  Bank
     * start and bank depth (in 32-bit words) are also set.
     */
    switch (bank)
    {
        case P7142_DDR_MEM_BANK0:
            P7142SetDdrMemCtrlParams (&ddrMemParams,
                                      P7142_DDR_MEM_BANK_0_READ_MODE);
            ddrMemParams.ddrMemBank0StartAddr = 0;
            ddrMemParams.ddrMemBank0Depth = (readLen/4);
        break;

        case P7142_DDR_MEM_BANK1:
            P7142SetDdrMemCtrlParams (&ddrMemParams,
                                      P7142_DDR_MEM_BANK_1_READ_MODE);
            ddrMemParams.ddrMemBank1StartAddr = 0;
            ddrMemParams.ddrMemBank1Depth = (readLen/4);
        break;

        case P7142_DDR_MEM_BANK2:
            P7142SetDdrMemCtrlParams (&ddrMemParams,
                                      P7142_DDR_MEM_BANK_2_READ_MODE);
            ddrMemParams.ddrMemBank2StartAddr = 0;
            ddrMemParams.ddrMemBank2Depth = (readLen/4);
        break;
    }

    /* apply the parameter table to the registers */
    P7142InitDdrMemRegs (&ddrMemParams, &(p7142Regs->BAR2RegAddr));

    /* start the transfer, wait for completion --------------------------- */

    /* read from DDR Memory Bank */
    P7142DmaStart(&(p7142Regs->BAR0RegAddr), P7142_DMA_CHAN_8);

    /* wait for interrupt completion */
    status = sem_wait(&ddrMemReadSem);
    if (status != 0)
    {
        sem_destroy(&ddrMemReadSem);
        PTK714X_DMAIntDisable(ddrDmaReadHandle);
        PTK714X_DMAFreeMem(ddrDmaReadHandle, &dmaBuf);
        PTK714X_DMAClose(hDev, ddrDmaReadHandle);
        return (8);
    }

    /* Sync Io Caches*/
    PTK714X_DMASyncIo(&dmaBuf);

    /* copy desired bytes from DMA buffer to data buffer */
    memcpy (dataBuf, dmaBuf.usrBuf, dataLen);


    /* clean up and exit ------------------------------------------------- */

    /* disable DDR memory */
    P7142_SET_DDR_MEM_MODE(p7142Regs->BAR2RegAddr.ddrMem.ddrMemCtrl,   \
                           P7142_DDR_MEM_DISABLE_MODE);

    /* FIFO disable */
    P7142_SET_FIFO_CTRL_FIFO_ENABLE(                                   \
        p7142Regs->BAR2RegAddr.ddrMemReadFifo.FifoCtrl,                \
        P7142_FIFO_DISABLE);

    /* clean up */
    sem_destroy(&ddrMemReadSem);
    PTK714X_DMAIntDisable(ddrDmaReadHandle);
    PTK714X_DMAFreeMem(ddrDmaReadHandle, &dmaBuf);
    PTK714X_DMAClose(hDev, ddrDmaReadHandle);

    return (0);
}

/////////////////////////////////////////////////////////////////////
uint32_t p7142::nextSimPulseNum(int chan) {

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
p7142::simWait() {
    // because the usleep overhead is large, sleep every 100 calls
   	if (_simPauseMS > 0) {
   	    if (!(_simWaitCounter++ % 100)) {
        	usleep((int)(_simPauseMS*1000));
    	}
    }
}
//////////////////////////////////////////////////////////////////////////////////
int
p7142::circuitBoardTemp() const {
    // Return a fixed value if simulating
    if (_simulate) {
        return(35);
    }

    LM83_VALUES     tempMonValues;
    /* read the LM83 temperature registers */
    Twsi_LM83GetValues((unsigned long)(_p7142Regs.BAR2RegAddr.twsiPort),
            P7142_TWSI_ADDR_LM83, &tempMonValues);
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
p7142::fpgaTemp() const {
    // Return a fixed value if simulating
    if (_simulate) {
        return(50);
    }

    LM83_VALUES     tempMonValues;
    /* read the LM83 temperature registers */
    Twsi_LM83GetValues((unsigned long)(_p7142Regs.BAR2RegAddr.twsiPort),
            P7142_TWSI_ADDR_LM83, &tempMonValues);
    // The D2 temperature sensor is on the signal processing FPGA.
    int temp = tempMonValues.D2Temp;
    // Pentek's library doesn't do the right thing unpacking temperatures
    // below 0. Correct that here.
    if (temp > 127)
    	temp -= 256;
    return(temp);
}

