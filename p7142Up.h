/*
 * p7142Up.h
 *
 *  Created on: Oct 12, 2010
 *      Author: burghart
 */

#ifndef P7142UP_H_
#define P7142UP_H_

#include <string>
#include <boost/thread/recursive_mutex.hpp>

namespace Pentek {

class p7142;

/// A p7142 upconverter
class p7142Up {
    public:
        /// Destructor
        virtual ~p7142Up();
        /// Write the baseband signal to ram.
        /// @param data The I and Q signal values
        /// @param n The number of values.
        virtual void write(int32_t* data, int n);
        /// Start the DAC output
        virtual void startDAC();
        /// Stop the DAC output
        virtual void stopDAC();
        /// @return The device path
        std::string upName();
        /// Are we simulating a real card?
        bool isSimulating() const;
        /// @return The dac clock in Hz.
        double sampleClockHz();

    protected:
        // Class p7142 is a friend; the intention is that construction of
        // p7142Up will happen only from there...
        friend class p7142;
        
        /// Constructor
        /// @param myP7142 A pointer to the owner p7142 object
        /// @param upName The name of the downconverter device, e.g. 0C
        /// @param sampleClockHz The DAC sample clock in Hz
        /// @param ncoFreqHz The NCO frequency in Hz
        /// @param mode The DAC CONFIG2 coarse mixer mode (See DAC5687 Data Sheet)
        p7142Up(p7142 * myP7142Ptr, std::string upName, double sampleClockHz, 
                double ncoFreqHz, char mode);

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
        /// @param fd The file descriptor of an open up conversion channel.
        /// @param reg The desired register number (0==VERSION, 1==CONFIG0, etc.)
        /// @returns The register value
        char getDACreg(int fd, int reg);
        /// Set the value of a DAC configuration register. Note that
        /// DAC registers are programmed 8 bits at a time, even though
        /// the RESET ioctl uses a 32 bit value parameter.
        /// @param fd The file descriptor of an open up conversion channel.
        /// @param reg The desired register number (0==VERSION, 1==CONFIG0, etc.)
        /// @param val The value for reg.
        void setDACreg(int fd, int reg, char val);
        /// Print the values of DAC configuration registers.
        /// @param fd The file descriptor of an open up conversion channel.
        void dumpDACregs(int fd);
        
        /// The P7142 which owns us...
        p7142 & _p7142;        
        /// The clock sample rate in hz
        double _sampleClockHz;
        /// The NCO frequency in HZ
        double _ncoFreqHz;
        /// The interpolation mode. 0=X2, 1=X4, 2=X4L, 3=X8
        long _interpMode;
        /// The up converter device name
        std::string _upName;
        /// The mem2 device name
        std::string _mem2Name;
        /// The up convertor file descriptor
        int _upFd;
        /// The depth of mem2 in 4 byte words. mem2 will contain the 
        /// DAC signal
        long _mem2depth;
        /// Mutex for thread safety
        mutable boost::recursive_mutex _mutex;

};

} // end namespace Pentek

#endif /* P7142UP_H_ */
