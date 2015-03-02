#ifndef P7142_H_
#define P7142_H_

#include "DDCregisters.h"

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <semaphore.h>
#include <csignal>
#include <vector>
#include <queue>
#include <map>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

#ifdef OPT_428
#include <ptk7142_428.h>
#else
#include <ptk7142.h>
#endif

#include <7142.h>

#include <boost/thread/recursive_mutex.hpp>

namespace Pentek {
	class p7142Up;
	class p7142Dn;
	class p7142;

	/// This structure holds user data associated with each
	/// dma channel. A pointer to this structure is registered
	/// with WinDriver, and that pointer is delivered to the
	/// dma handler for each dma transfer complete interrupt.
	struct DmaHandlerData {
	    int chan;
	    Pentek::p7142* p7142;

	};

	/// A structure to manage details about the downconverters
	/// that are active.
	struct DownconverterInfo {
		/// The downconverter
		p7142Dn* _dn;
		/// The last pulsenumber granted for this downconverter
		uint32_t _pulseNum;
	};

    /// The maximum pulse number in a pulse tagger tag
    static const int32_t MAX_PT_PULSE_NUM = 0x3FFFFFFF;

    /// The maximum pulse number in a coherent integrator tag
    static const int32_t MAX_CI_PULSE_NUM = 0xFFFFFF;

    /// Foundation class for a p7142 digital transceiver card.
    /// Card configuration and interaction are managed via the
    /// ReadyFlow C API.
    ///
    /// <h1>Overview</h1>
    /// ReadyFlow uses WinDriver to provide access to the PCI
    /// interface. There are two activities: reading and writing to the
    /// Pentek PCI register space, and interacting with DMA transfers
    /// from the card.
    ///
    /// The PCI address space is mapped directly to registers in the
    /// Pentek PCI FPGA and the signal FPGA. Controlling the Pentek functions
    /// in this manner logical, although fairly involved.
    ///
    /// The DMA system functions by allocating a collection of DMA buffers,
    /// which will be directly written into by the Pentek card. When a transfer
    /// is completed, a dma "interrupt" is triggered, which asynchronously
    /// executes a C handler function in the user process space. The DMA
    /// is configured to loop continuously through 4 DMA buffers.
    ///
    /// A system of buffers and logic is used to capture the delivered data,
    /// buffer it through system slowdowns, and feed it out to data read requests.
    ///
    /// <h1>The Buffer scheme</h1>
    /// Three buffer systems are used:
    /// @li The WinDriver DMA buffers
    /// @li A circular list of buffers for intermediate storage
    /// @li A pending read buffer, used to accumulate bytes to satisfy read requests.
    ///
    /// @li Each DMA interrupt causes dmaIntHandler() to be entered.
    /// @li dmaIntHandler() calls p7142::dmaInterrupt(), which transfers
    /// the data from the dma buffer to a buffer in the circular buffer list.
    /// @li p7142::read() will attempt to copy bytes from the buffers on the circular
    /// buffer list to the pending read buffer. If the pending read buffer
    /// has enough bytes to satisfy the the request, they will be transferred to
    /// the caller buffer and the read will return.
    /// @li Otherwise, p7142::read() will block until another buffer is
    /// added in the circular buffer list.
    ///
    /// The circular buffer list allows for a large number of DMA transfers to
    /// be saved in times of heavy system load. (However, it's not clear that
    /// the DMA interrupts will even be delivered in times of heavy load).
    ///
    /// The pending read buffer is not strictly necessary. read() requests
    /// could be filled incrementally out of the circular buffers. But the pending
    /// read buffer makes the book keeping much easier, and the logic simpler.
    /// If performance appears to be an issue, this could be one area to look at
    /// for a redesign.
    ///
    /// <h1>Lineage</h1>
    /// The p7142 class was mostly adapted from the ReadyFlow example
    /// programs. Its C ancestry is obvious. There are many references
    /// to the DAC5686. Our Pentek cards are customized, where the
    /// DAC5686 was replaced by the DAC5687. We retained the DAC5686
    /// nomenclature so as to match the ReadyFlow usage.
    ///
    /// <h1>ReadyFlow</h1>
    /// ReadyFlow seems to have two methods of configuration manipulation. At
    /// the highest level, a structure with fields corresponding
    /// to many of the GateFLow options is provided for each subsystem. The
    /// subsystems are PCI, DMA, board, input (down conversion), output (up conversion)
    /// and DAC.
    ///
    /// The user initializes a structure using a SetDefault() call,
    /// e.g. P7142SetPCIDefaults(). Individual fields in the structure are
    /// modified to configure for non-default behavior. The structure is then
    /// written to the board registers with an InitRegs() call, e.g.
    /// P7142InitPCIRegs().
    ///
    /// ReadyFlow also provides get/set macros for manipulating individual
    /// control bits. So, once a section has had a complete initial configuration
    /// using P7142InitInitRegs(), the  macros can be used to access individual
    /// control bits as needed. In fact, the InitRegs() functions are built
    /// around these macros.
    ///
    /// The approach taken here will be to initialize the entire board in
    /// p7142. The associated p7142Dn and p7142Up classes will use the
    /// ReadyFlow macros as needed.
	///
	/// <h1>Simulation</h1>
	/// The sd3c firmware can be configured to prepend pulse numbers
	/// to each beam of data. In the ddc4 and ddc8 configurations, the separate
	/// channels on the card operate at the same data rate, and are synchronized
	/// such that the pulse numbers at a given time will be identical among
	/// channels.
	///
	/// Since the p7142dn downconverters are individually instantiated, they
	/// are unaware of each other and cannot coordinate pulse numbers when
	/// operating in simulation mode. p7142 provides a mechanism for achieving this.
	/// The downconverters consume pulse numbers provided by nextSimPulseNum(). This routine keeps
	/// track of successive calls by each downconverter, and makes sure that they are receiving
	/// coordinated pulse numbers. If a requesting downconverter is getting ahead of
	/// its peers, nextSimPulseNum() will block that thread until all of the
	/// downconverters have requested the next pulse number.
	///
	/// nextSimPulseNum() will also throttle the production of pulse numbers, by
	/// sleeping for a specified time. It does this after every 100 pulse number
	/// requests, since sleep() is an expensive habit.
	class p7142 {

		public:
            /// A P7142 card has 4 receive channels available.
            static const int P7142_NCHANNELS = 4;
            /// (Suggested) time to sleep after P7142 ioctl calls, in microseconds
            static const int P7142_IOCTLSLEEPUS = 100;
            
            /// @brief Construct a p7142 associated with the next available 
            /// Pentek 7142 card in the system. The access order for
            /// cards is system-dependent, and specifically affected by cards'
            /// location on the PCI bus(es).
            /// @param simulate Set true if we operate in simulation mode.
            /// @param simPauseMS The number of milliseconds to wait,after
            /// every 100 requests for a simulated pulse number.
            /// @param useFirstCard If true, use the first card in the system. Otherwise,
            /// the next card will be searched for.
            p7142(bool simulate = false, double simPauseMS = 50.0, bool useFirstCard=false);
			/// Destructor.
			virtual ~p7142();
            /// @brief Tell if the P7142 is successfully configured and ready
            /// to operate.
            /// @return true if the P7142 is successfully configured and ready
            /// to operate, false otherwise.
            virtual bool ok() const { return _isReady; }
            /// @brief Return true iff we're in simulation mode.
            /// @return true iff we're in simulation mode.
            bool isSimulating() const { return _simulate; }
            /// Turn on the global FIFO gate enable.
            void enableGateGen();
            /// Turn off the global FIFO gate enable
            void disableGateGen();

            /// @brief Construct and add a downconverter for one of our receiver channels.
            /// @param chanId The channel identifier (used to select /dn/*B)
    		/// @param dmaDescSize DMA descriptor size to use for this channel. This
    		/// is the amount of data written to DMA by the Pentek before an interrupt
    		/// is generated indicating data should be read.
            /// @param bypassdivrate The bypass divider (decimation) rate
            /// @param simWavelength The wave length, in timeseries points, for the
            /// simulated data. See p7142Dn::read().
            /// @param sim4bytes Create 4 byte instead of 2 byte integers when
            /// in simulation mode. This simulates the output of the coherent 
            /// integrator.
            virtual p7142Dn* addDownconverter(int chanId, uint32_t dmaDescSize,
                    int bypassdivrate = 1, int simWavelength = 5000, 
                    bool sim4bytes = false);
            
            /// @brief Construct and add an upconverter for our device.
            /// @param sampleClockHz The DAC sample clock in Hz
            /// @param ncoFreqHz The NCO frequency in Hz
            /// @param mode The DAC CONFIG2 coarse mixer mode (See DAC5687 Data Sheet)
            virtual p7142Up* addUpconverter(
                    double sampleClockHz, double ncoFreqHz, char mode);
            
            /// @brief Get the temperature of the signal-processing FPGA chip
            /// on the P7142, deg C
            ///
            /// The temperature returned is the D2 temperature from the LM83
            /// sensor on the P7142's board. See Section 6.3.1 and Table 6-4 in
            /// the Pentek Operating Manual.
            /// @return the temperature of the signal-processing FPGA chip
            /// on the P7142, deg C
            int fpgaTemp() const;

            /// @brief Get the P7142 circuit board temperature, deg C
            ///
            /// The temperature returned is the D3 temperature from the LM83
            /// sensor on the P7142's board. See Section 6.3.1 and Table 6-4 in
            /// the Pentek Operating Manual.
            /// @return the P7142 circuit board temperature, deg C
            int circuitBoardTemp() const;

            /// get the index of the card in use
            int getCardIndex() const { return _cardIndex; }
            
            /// @brief Return a pointer to the downconverter for the given
            /// channel, or NULL if there is no downconverter for the channel.
            /// @return a pointer to the downconverter for the given
            /// channel, or NULL if there is no downconverter for the channel.
            const p7142Dn* downconverter(int chanId) const {
                if (_downconverters.find(chanId) != _downconverters.end()) {
                    return(_downconverters.find(chanId)->second._dn);
                } else {
                    return(NULL);
                }
            }
            
            /// @brief Return a pointer to this card's upconverter, or null if
            /// there is no upconverter.
            /// @return a pointer to this card's upconverter, or null if
            /// there is no upconverter.
            const p7142Up* upconverter() const {
                return(_upconverter);
            }

            // We make our associated downconverter and upconverter classes 
            // friends so that they have access to BAR registers, etc.
            // methods, etc.
            friend class p7142Dn;
            friend class p7142Up;

		protected:
            /// Add (or replace) a downconverter on our list. If the 
            /// downconverter is associated with a channel for which we already
            /// have a downconverter, the existing downconverter for that
            /// channel will be deleted. This object assumes ownership of the 
            /// incoming downconverter.
            /// @param downconverter the downconverter to be added.
            void _addDownconverter(p7142Dn* downconverter);
            
            /// Add (or replace) our upconverter. If we already have an
            /// upconverter, it will be deleted. This object assumes ownership 
            /// of the incoming upconverter.
            /// @param upconverter the upconverter to be added.
            void _addUpconverter(p7142Up* upconverter);
            
            uint32_t nextSimPulseNum(int chan);

            /// Perform a FIOREGGET ioctl call to the control device for the 
            /// given address. The resulting value is returned.
            /// @param addr The address of the register to get.
            /// @return The value in the selected register.
            unsigned int _regget(unsigned int addr);

            /// Reset the digital clock managers on the FPGA. Necessary since
            /// some of the new DCMs we add in the firmware use the
            /// CLKFX output, which won't lock at startup. <em>Downconverters
            /// must call this method whenever they change their clock source
            /// via the CLKSRCSET ioctl!</em>
            void _resetDCM();
            /// Initialize the ReadyFlow library.
            bool _initReadyFlow();
            /// Create a random number, with Gaussian distribution about a 
            /// selected mean and with selected standard deviation.
            /// Useful for simulation
            /// @param[in] mean mean value of the Gaussian distribution
            /// @param[in] stdDev standard deviation of the Gaussian distribution
            /// @return the generated random number
            static double _gauss(double mean, double stdDev);
            /// Set the dma buffer and interrupt buffersize. The
            /// buffersize must be at least 2x the interrupt buffer size.
            /// Perhaps it should be even more?
            /// @param fd file descriptor
            /// @param intbufsize Interrupt buffer size.
            /// @param bufN The driver buffer will be this factor times intbufsize
            /// @return 0 on success, -1 on failure.
            static int _bufset(int fd, int intbufsize, int bufN);
            /// Configure the board parameters, in p7142BoardParams
            void _configBoardParameters();
            /// Configure the down conversion path parameters
            void _configInParameters();
            /// Configure the up conversion path parameters
            void _configOutParameters();
            /// Write to the selected memory bank.
            /// @param bank The selected bank -  0, 1 or 2
            /// @param buf Pointer to the buffer of bytes to be written.
            /// @param bytes The number of bytes to write.
            /// @return The number of bytes written. If an error occurs,
            /// minus one will be returned.
            int memWrite(int bank, int32_t* buf, int bytes);
            /// Read from the selected memory bank.
            /// @param bank The selected bank -  0, 1 or 2
            /// @param buf The data will be returned here.
            /// @param bytes The number of bytes to read.
            /// memwrite() will resize the vector as required.
            /// @return The number of bytes read. If an error occurs,
            /// minus one will be returned.
            int memRead(int bank, int32_t* buf, int bytes);
            /// Borrowed shamelessly from dmem_dac.c in the ReadyFlow examples
            /// (but heavily modified to work with shorter write lengths and
            /// write lengths that are not a multiple of 32 bytes)
            ///
            /// Write data to the selected DDR memory bank, using DMA Channel 7.
            /// @param p7142Regs Pointer to the 7142 register address table
            /// @param bank Memory bank to write to. Use defines P7142_DDR_MEM_BANK0,
            /// P7142_DDR_MEM_BANK1 or P7142_DDR_MEM_BANK2
            /// @param dataLen Number of bytes to write. Yes, BYTES
            /// @param dataBuf Pointer to the data buffer containing the data
            /// @param hDev The 7142 Device Handle
            /// @returns <br>
            /// 0 - successful <br>
            /// 1 - invalid bank number  <br>
            /// 3 - bank depth extends past the end of the DDR bank <br>
            /// 4 - DMA channel failed to open <br>
            /// 5 - DMA buffer allocation failed <br>
            /// 6 - semaphore creation failed <br>
            /// 7 - semaphore wait timed out <br>
            /// 8 - verification of written memory failed
            int _ddrMemWrite (P7142_REG_ADDR* p7142Regs,
                             unsigned int    bank,
                             unsigned int    dataLen,
                             unsigned int   *dataBuf,
                             PVOID           hDev);
            /// Lifted shamelessly from dmem_dac.c in the readyflow examples
            ///
            /// Read data to the selected DDR memory bank, using DMA Channel 7.
            /// @param p7142Regs Pointer to the 7142 register address table
            /// @param bank Memory bank to write to. Use defines P7142_DDR_MEM_BANK0,
            /// P7142_DDR_MEM_BANK1 or P7142_DDR_MEM_BANK2
            /// @param dataLen Number of bytes to read.
            /// @param dataBuf Pointer to the data buffer containing the data
            /// @param hDev The 7142 Device Handle
            /// @returns <br>
            /// 0 - successful <br>
            /// 1 - invalid bank number  <br>
            /// 3 - bank depth extends past the end of the DDR bank <br>
            /// 4 - DMA channel failed to open <br>
            /// 5 - DMA buffer allocation failed <br>
            /// 6 - semaphore creation failed <br>
            /// 7 - semaphore wait timed out
            int _ddrMemRead (P7142_REG_ADDR *p7142Regs,
                            unsigned int    bank,
                            unsigned int    dataLen,
                            unsigned int   *dataBuf,
                            void*           hDev);
            /// Each 100 calls, sleep for simPauseMS milliseconds.
            void simWait();
            
            /// Keep track of the PCI slot of the last instantiated p7142.
            /// This is needed for calls to ReadyFlow's PTK714X_DeviceFindAndOpen() 
            /// function. This variable starts at -2, and changes for each
            /// p7142 we instantiate. (This change actually occurs at each call 
            /// from the constructor to PTK714X_DeviceFindAndOpen()).
            static DWORD _Next7142Slot;
            /// Keep track of how many 7142 cards we have open (i.e., how
            /// many instances of this class are there so far).
            static uint16_t _NumOpenCards;

            /// ReadyFlow device descriptor.
            void* _deviceHandle;
            /// Index for this instance of p7142.
            uint16_t _cardIndex;
            /// ReadyFlow PCI BAR0 base address.
            DWORD                 _BAR0Base;
            /// ReadyFlow PCI BAR2 base address.
            DWORD                 _BAR2Base;
            /// ReadyFlow PCI slot number.
            DWORD                 _pciSlot;
            /// ReadyFlow module identifier.
            unsigned int          _moduleId;
            /// ReadyFlow 7142 register addresses in PCI space.
            P7142_REG_ADDR        _p7142Regs;
            /// ReadyFlow parameters for PCI configuration.
            P7142_PCI_PARAMS      _p7142PciParams;
            /// ReadyFlow parameters for DMA configuration.
            P7142_DMA_PARAMS      _p7142DmaParams;
            /// ReadyFlow parameters for overall board configuration.
            P7142_BOARD_PARAMS    _p7142BoardParams;
            /// ReadyFlow parameters for the down conversion path configuration.
            P7142_INPUT_PARAMS    _p7142InParams;
            /// ReadyFlow parameters for the up conversion path configuration.
            P7142_OUTPUT_PARAMS   _p7142OutParams;
            /// ReadyFlow parameters for the DDR memory.
            P7142_DDR_MEM_PARAMS  _p7142MemParams;
            /// ReadyFlow parameters for DAC configuration.
            P7142_DAC5686_PARAMS  _p7142Dac5686Params;
            /// The PCI address of the GateFlow gate generation control register.
            volatile unsigned int *_gateGenReg;
            /// set true if in simulation mode
            bool _simulate;
            /// recursive mutex which provides us thread safety.
            mutable boost::recursive_mutex _p7142Mutex;
            /// True if device is opened and accessible
            bool _isReady;
            /// The down converters attached to this device.
            /// The container is indexed by the channel number.
            std::map<int, DownconverterInfo> _downconverters;
            /// The upconverter attached to this device
            p7142Up* _upconverter;
            /// The simulation pulse number.
            uint32_t _simPulseNum;
            /// The number of downconverters waiting for a pulse number
            unsigned int _waitingDownconverters;
            /// The condition variable used to block downconverters who
            /// need to wait for peers to catch up in consuming simulated
            /// pulse numbers.
            boost::condition_variable _simPulseNumCondition;
            /// Mutex used with the simulated pulse number condition variable
            mutable boost::mutex _simPulseNumMutex;
            /// The sim wait counter, which controls how often simWait() will
            /// actually sleep.
            unsigned int _simWaitCounter;
            /// The number of milliseconds that simWait() will sleep
            double _simPauseMS;

	};

} // end namespace Pentek

#endif /*P7142_H_*/
