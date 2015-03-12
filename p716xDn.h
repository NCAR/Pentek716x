#ifndef P716XDN_H_
#define P716XDN_H_

#include <boost/thread/recursive_mutex.hpp>
#include <ptk_osdep.h>  // include this before other ReadyFlow headers
#include "p716x.h"

namespace Pentek {

/// @brief A p716x downconverter.
/// This class reads and controls downconversion for one receiver channel of a
/// P716x transceiver card.
class p716xDn {
/// Define the number of DMA buffers available for the Readyflow DMA interrupt handler.
#define N_READYFLOW_DMA_BUFFERS 100

public:
    /// Constructor
    /// @param p716x a pointer to the owner p716x object
    /// @param chanId The channel identifier (used to select /dn/*B)
    /// @param dmaDescSize DMA descriptor size to use for this channel. This
    /// is the amount of data written to DMA by the Pentek before an interrupt
    /// is generated indicating data should be read.
    /// @param decimation The decimation rate
    /// @param simWaveLength The wave length, in timeseries points, for the
    /// simulated data. See read().
    /// @param sim4bytes Create 4 byte instead of 2 byte integers when
    /// in simulation mode. This simulates the output of the coherent integrator.
    /// @param internalClock Set true if the internal clock should be
    /// used instead of an external clock source.
    p716xDn(p716x* p716x,
            uint16_t chanId,
            uint32_t dmaDescSize,
            int decimation = 1,
            int simWaveLength = 5000,
            bool sim4bytes = false,
            bool internalClock = false);

    /// Destructor
    virtual ~p716xDn();

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

    /// @brief Return the ADC channel id (0-2) for this downconverter.
    /// @return the ADC channel id (0-2) for this downconverter.
    uint16_t chanId() const { return _chanId; }

    /// @brief Are we simulating existence of a real P716x card?
    /// @return true iff we are simulating a P716x card rather than using
    /// a real one.
    bool isSimulating() const { return _p716x.isSimulating(); }
        

    protected:
        /// @brief Start Pentek writing to DMA for this channel.
        void _start();
        
        /// @brief Stop Pentek from writing to DMA for this channel.
        void _stop();
        
        /// @brief This static function is called by WinDriver each time a 
        /// DMA descriptor transfer is completed by an ADC channel.
        ///
        /// The user data (pData) is a pointer to the instance of p716xDn
        /// responsible for the channel with incoming data. The instance's
        /// _dmaSemaphorePost() method is called to note that DMA data are 
        /// available to read.
        ///
        /// DMA interrupts are cleared by the Kernel Device Driver.
        ///
        /// DMA interrupts are enabled when this routine is executed.
        /// @param deviceHandle the device handle of the source Pentek card
        /// @param intType the type of the incoming interrupt
        /// @param instancePointer pointer to p716xDn instance which should
        /// handle the interrupt
        /// @param pIntResult pointer to the interrupt results structure
        static void _StaticIntHandler(PVOID deviceHandle,
                                      unsigned long intType,
                                      PVOID instancePointer,
                                      PTK716X_INT_RESULT *pIntResult)
        {
            // Just increment the "DMA ready" semaphore on the downconverter
            // which should get the new data.
            p716xDn * downconverter = static_cast<p716xDn *>(instancePointer);
            downconverter->_dmaSemaphorePost();
        }
        
        /// @brief Function which increments the "DMA data available" semaphore.
        void _dmaSemaphorePost();
        
        /// @brief Return the "DMA complete" semaphore number for this 
        /// downconverter.
        int _dmaCompleteSemNum() {
            // We just use the ADC channel number
            return(_chanId);
        }

        /// @brief Return the thread number for our DMA reading thread.
        int _dmaThreadNum() {
            // We just use the ADC channel number
            return(_chanId);
        }
        
        /// @brief Static function which just calls the given instance's 
        /// _dmaThreadMainLoop() method.
        /// @param instancePointer pointer to an instance of p716xDn
        static void _StaticDmaThreadMainLoop(void * instancePointer) {
            // Just call the _dmaThreadMainLoop() method on the given dow
            p716xDn * downconverter = static_cast<p716xDn *>(instancePointer);
            downconverter->_dmaThreadMainLoop();
        }
        
        /// @brief This method is the execution loop for a thread which handles 
        /// reading of data from the ADC DMA. It simply waits for the "DMA 
        /// data available" semaphore, reads data into a local buffer, and 
        /// repeats.
        void _dmaThreadMainLoop();
        
        /// @brief Create the thread which will handle reading of data from
        /// DMA delivered by our ADC channel.
        void _initDma();

        /// @brief Apply the ADC channel parameter table to the registers on
        /// the card
        void _applyAdcParams();

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
        
        /// The P716x which owns us...
        p716x& _p716x;
        
        /// Receiver channel number (0-3)
        unsigned int _chanId;
        
        /// ADC parameters for this channel
        P716x_ADC_CHAN_PARAMS _adcParams;

        /// Struct holding Pentek's OS-dependent thread, semaphore, and mutex
        /// information.
        IFC_ARGS _ifcArgs;
        
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
        PTK716X_DMA_HANDLE*   _dmaHandle;
        
        /// ReadyFlow DMA buffer address pointers, one for each of the 
        /// four "descriptors" the DMA cycles through
        static const uint16_t N_DMA_DESCRIPTORS = 4;
        PTK716X_DMA_BUFFER    _dmaBuf[N_DMA_DESCRIPTORS];
        
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
        
        /// Boolean set to true when exiting
        bool _exiting;
};

}   // end namespace Pentek

#endif /* P716XDN_H_ */
