#ifndef P7142UP_H_
#define P7142UP_H_

#include "p7142.h"
#include <string>
#include <boost/thread/recursive_mutex.hpp>

namespace Pentek {

/// A p7142 upconverter. This class manges configuration and
/// operation of the D/A section of the p7142. The device
/// is configured for data to be routed from memory 2 to
/// the DC FIFO and from there to the TI 5687 DAC.
///
/// The DAC is very sophisticated. ReadyFlow did not come with
/// support for this particular chip. It was adapted to provide
/// most of the ReadyFlw API, but some functions were not
/// implmented. For that reason, you will see a mixture of ReadyFlow
/// macros and direct manipulation of the DAC5687 registers. In
/// general, anything that does not involve configuration of the DAC
/// itself can be handlesd by ReadyFlow calls.
///
/// See page 20 of the TI DAC5687 data sheet
/// for an explanation of the modes of operation. The mode is determined
/// by the combination of interpolation rate, fine mixer, quadrature
/// modulation phase correction and coarse mixer.
///
/// We allow the user to specify the DAC interpolation choices of
/// X2, X4, X4L and X8. The fine mixer (i.e. NCO) and coarse mixer
/// are always enabled.
class p7142Up {
    public:
        /// Destructor
        virtual ~p7142Up();
        /// Write the baseband signal to memory 2 on the Pentek.
        /// @param data The I and Q signal values
        /// @param n The number of 4 byte values.
        virtual void write(int32_t* data, int n);
        /// enable the DAC output. Actuall values from memory 2 will not
        /// be fed to the DAC until txgate enables the DAC FIFO.
        virtual void startDAC();
        /// Stop the DAC output.
        virtual void stopDAC();
        /// @return The device path
        std::string upName();
        /// Are we simulating a real card?
        bool isSimulating() const;
        /// @return The dac clock in Hz.
        double sampleClockHz();

        // Class p7142 is a friend; the intention is that construction of
        // p7142Up will happen only from there...
        friend class p7142;
        
        /// Constructor
        /// @param p7142ptr A pointer to the owner p7142 object
        /// @param sampleClockHz The DAC sample clock in Hz
        /// @param ncoFreqHz The NCO frequency in Hz
        /// @param cmMode The DAC CONFIG2 coarse mixer mode (See DAC5687 Data Sheet)
        p7142Up(p7142 * p7142ptr, double sampleClockHz,
                double ncoFreqHz, char cmMode);

    protected:
        /// Initialize the DAC hardware. Register level calls are
        /// made to manipulate the internal DAC configuration registers.
        bool initDAC();

        /// Calculate the NCO frequency control bytes. It is dependent upon the
        /// the mode (_interp) and the sample clock rate.
        /// @param fNCO The desired NCO frequency in Hz
        /// @param fDAC The DAC clock rate
        /// @param nco_freq_0 Byte 0 of the nco frequency parameter is returned here
        /// @param nco_freq_1 Byte 1 of the nco frequency parameter is returned here
        /// @param nco_freq_2 Byte 2 of the nco frequency parameter is returned here
        /// @param nco_freq_3 Byte 3 of the nco frequency parameter is returned here
        void ncoConfig(double fNCO, double fDAC, char& nco_freq_0, char& nco_freq_1, char& nco_freq_2, char& nco_freq_3);
        /// Fetch the value of a DAC configuration register.
        /// @param reg The desired register number (0==VERSION, 1==CONFIG0, etc.)
        /// @returns The register value
        char getDACreg(int reg);
        /// Set the value of a DAC configuration register. Note that
        /// DAC registers are programmed 8 bits at a time.
        /// @param reg The desired register number (0==VERSION, 1==CONFIG0, etc.)
        /// @param val The value for reg.
        void setDACreg(int reg, char val);
        /// Print the values of DAC configuration registers.
        void dumpDACregs();
        
        /// The P7142 which owns us...
        p7142 & _p7142ptr;
        /// The clock sample rate in hz
        double _sampleClockHz;
        /// The NCO frequency in HZ
        double _ncoFreqHz;
        /// The coarse mixer mode
        char _cmMode;
        /// The interpolation mode. 0=X2, 1=X4, 2=X4L, 3=X8
        long _interp;
        /// The depth of mem2 in 4 byte words. mem2 will contain the 
        /// DAC signal
        long _mem2depthWords;
        /// Mutex for thread safety
        mutable boost::recursive_mutex _mutex;

};

} // end namespace Pentek

#endif /* P7142UP_H_ */
