#ifndef P7142DN_H_
#define P7142DN_H_

#include <string>
#include <boost/thread/recursive_mutex.hpp>
#include "p7142.h"

namespace Pentek {

/// @brief A p7142 downconverter.
/// This class reads and controls downconversion for one receiver channel of a
/// P7142 transceiver card.
class p7142Dn {
/// Define the number of DMA buffers available for the Readyflow DMA interrupt handler.
#define N_READYFLOW_DMA_BUFFERS 100

public:
    /// Constructor
    /// @param p7142 a pointer to the owner p7142 object
    /// @param chanId The channel identifier (used to select /dn/*B)
    /// @param dmaDescSize DMA descriptor size to use for this channel. This
    /// is the amount of data written to DMA by the Pentek before an interrupt
    /// is generated indicating data should be read.
    /// @param bypassdivrate The bypass divider (decimation) rate
    /// @param simWaveLength The wave length, in timeseries points, for the
    /// simulated data. See read().
    /// @param sim4bytes Create 4 byte instead of 2 byte integers when
    /// in simulation mode. This simulates the output of the coherent integrator.
    /// @param internalClock Set true if the internal clock should be
    /// used instead of an external clock source.
    p7142Dn(p7142* p7142,
            int chanId,
            uint32_t dmaDescSize,
            int bypassdivrate = 1,
            int simWaveLength = 5000,
            bool sim4bytes = false,
            bool internalClock = false);

        /// Destructor
        virtual ~p7142Dn();
        /// Read bytes.
        /// @param buf read bytes into this buffer
        /// @param bufsize The number of bytes to read.
        /// @return The actual number of bytes read
        virtual int read(char* buf, int bufsize);
        /// Return the number of bytes read by this downconverter
        /// since the last call to bytesRead().
        /// @return the number of bytes read by this downconverter
        /// since the last call to bytesRead().
        long bytesRead();
        /// Are we using the card's internal clock?
        /// @return true iff this channel using the card's internal clock
        bool usingInternalClock() const;
        /// Get the current bypass divider decimation factor
        /// @return the current bypass divider decimation factor
        int bypassDivider() const;
        ///
        /// Set the bypass divider decimation factor
        /// @param bypassdiv the desired bypass divider decimation factor
        /// @return true if decimation is set successfully
        bool setBypassDivider(int bypassdiv) const;
        ///
        /// Return the channel id (0-3) for this downconverter.
        /// @return the channel id (0-3) for this downconverter.
        int chanId() const { return _chanId; }
        /// Are we simulating existence of a real P7142 card?
        /// @return true iff we are simulating a P7142 card rather than using
        /// a real one.
        bool isSimulating() const { return _p7142.isSimulating(); }
        

    protected:
        /// @brief Start Pentek writing to DMA for this channel.
        void _start();
        /// @brief Stop Pentek from writing to DMA for this channel.
        void _stop();
        /// @brief This static function is called by WinDriver each time a 
        /// DMA descriptor transfer is completed by an ADC channel.
        ///
        /// The user data (pData) is a pointer to the instance of p7142Dn
        /// responsible for the channel with incoming data. The 
        /// p7142Dn::_dmaInterrupt() method is called to handle the actual
        /// processing of the DMA transfer.
        ///
        /// DMA interrupts are cleared by the Kernel Device Driver.
        ///
        /// DMA interrupts are enabled when this routine is executed.
        /// @param dmaHandle The DMA handle for the source channel
        /// @param dmaChannel - number of the DMA channel generating the interrupt(0-3)
        /// @param pData - Pointer to user defined data
        /// @param pIntResult - Pointer to the interrupt results structure
        static void _staticDmaHandler(
                PVOID dmaHandle,
                unsigned int dmaChannel,
                PVOID pData,
                PTK714X_INT_RESULT *pIntResult);
        /// @brief This method is called from the static method 
        /// _adcDmaIntHandler(), indicating that DMA data are ready for reading 
        /// for this downconverter's channel. Data are read from DMA and 
        /// inserted in _dmaDescSize chunks into the _filledBuffers queue.
        /// According to the ReadyFlow notes, DMA interrupts are disabled
        /// while _dmaIntHandler() is executing, and so they will be disabled 
        /// as well while this method is executing.
        void _dmaInterrupt();

        /// @brief Initialize the fifo and DMA for a selected ADC channel.
        /// The fifos will be configured to be gated by a gate signal,
        /// which should be the global gate coming from the gate generator
        /// register. That signal is controlled by p7142::disableGateGen()
        /// and p7142::enableGateGen().
        void _initFifoAndDma();

        /// @brief Read bytes from the ADC channel. If no data are
        /// available, the thread will be blocked. The request will not
        /// return until the exact number of requested bytes can
        /// be returned.
        /// @param buf Buffer to receive the bytes..
        /// @param bytes The number of bytes.
        /// @return The number of bytes read. If an error occurs, minus
        /// one will be returned.
        int _read(char* buf, int bytes);
        /// @brief The _simulatedRead() mimics _read(), but returns simulated
        /// data rather than data actually obtained from the Pentek card.
        /// A noisy sine wave with wavelength of _simWaveLength gates will be 
        /// synthesized.
        /// @see _read()
        /// @param buf Buffer to receive the bytes..
        /// @param bytes The number of bytes.
        /// @return The number of bytes "read". If an error occurs, minus
        /// one will be returned.
        virtual int _simulatedRead(char* buf, int bytes);
        /// The P7142 which owns us...
        p7142& _p7142;
        /// Receiver channel number (0-3)
        int _chanId;
        /// The number bytes in each DMA descriptor. This is the data interval
        /// between interrupts telling us to read data from DMA.
        const int _DmaDescSize;
        /// The number of bytes read since the last call to bytesRead()
        long _bytesRead;
        /// The wavelength for simulated data
        unsigned int _simWaveLength;
        /// The counter for keeping track of the current phase during simulation
        unsigned int _angleCount;
        /// True if simulation is supposed to produce 4 byte integers
        bool _sim4bytes;
        /// Mutex for thread safety
        mutable boost::recursive_mutex _mutex;
        /// ReadyFlow DMA handle
        PTK714X_DMA_HANDLE*   _dmaHandle;
        /// ReadyFlow DMA buffer address pointers, one for each of the 
        /// four "descriptors" the DMA cycles through
        PTK714X_DMA_BUFFER    _dmaBuf[4];
        /// true if an AD channel is running
        bool _adcActive;
        /// Queue of free buffers available to hold data read from DMA
        /// Each buffer will be of length _dmaDescSize
        std::queue<char*> _freeBuffers;
        /// Queue of filled buffers
        std::queue<char*> _filledBuffers;
        /// Mutex to control access to _freeBuffers and _filledBuffers
        boost::mutex _bufMutex;
        /// Condition variable which will activate when data are available
        /// in _filledBuffers
        boost::condition_variable _dataReadyCondition;
        /// This is a vector used for temporary storage to satisfy read 
        /// requests. To avoid ongoing resizing, we allocate it to its full 
        /// length (2*_dmaBufSize). Bytes are added to the end of this buffer, 
        /// and sucked out of the beginning to satisfy read requests.,
        /// _readBufAvail tracks how many bytes are available in the buffer.
        /// _readBufOut is the index of the next byte available in the buffer.
        std::vector<char> _readBuf;
        /// The number of bytes available in the _readBuf.
        int _readBufAvail;
        /// The next available byte in _readBuf.
        unsigned int _readBufOut;
        /// The next DMA descriptor to read.
        /// The descriptor chain is 4 buffers long, and then it wraps back.
        uint16_t _nextDesc;
};

}   // end namespace Pentek

#endif /* P7142DN_H_ */
