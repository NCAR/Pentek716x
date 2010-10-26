/*
 * p7142Dn.h
 *
 *  Created on: Oct 4, 2010
 *      Author: burghart
 */

#ifndef P7142DN_H_
#define P7142DN_H_

#include <string>
#include <boost/thread/recursive_mutex.hpp>

namespace Pentek {

class p7142;

/*!
 * @brief A p7142 downconverter.
 * This class reads and controls downconversion for one receiver channel of a 
 * P7142 transceiver card.
 */ 
class p7142Dn {
    public:
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
        /// Return The number of overrun, or underrun samples and clear
        /// the over/under-run counter. Return -1 if unable to get this 
        /// information, and set _ok to false.
        /// @return The number of overrun, or underrun samples and clear
        /// the over/under-run counter. Return -1 if unable to get this 
        /// information, and set _ok to false.
        virtual int overUnderCount();
        /// Return the device path for the downconverter.
        /// @return The device path
        std::string dnName();
        /// Return the file descriptor for the downconverter device.
        /// @return The file descriptor.
        int fd();
        /// flush the I/O buffers and hardware fifos
        void flush();
        /// Are we using the card's internal clock?
        /// @return true iff this channel using the card's internal clock
        bool usingInternalClock() const;
        /// Get the current bypass divider decimation factor
        /// @return the current bypass divider decimation factor
        int bypassDivider() const;
        /**
         * Set the bypass divider decimation factor
         * @param bypassdiv the desired bypass divider decimation factor
         * @return true if decimation is set successfully
         */
        bool setBypassDivider(int bypassdiv) const;
        /**
         * Return the channel id (0-3) for this downconverter.
         * @return the channel id (0-3) for this downconverter.
         */
        int chanId() const { return _chanId; }
        /**
         * Are we simulating existence of a real P7142 card?
         * @return true iff we are simulating a P7142 card rather than using
         * a real one.
         */
        bool isSimulating() const;
        
    protected:
        // Class p7142 is a friend; the intention is that construction of
        // p7142Dn will happen only from there...
        friend class p7142;
        
        /// Constructor
        /// @param myP7142Ptr a pointer to the owner p7142 object
        /// @param chanId The channel identifier (used to select /dn/*B)
        /// @param bypassdivrate The bypass divider (decimation) rate
        /// @param simulate Set true if we operate in simulation mode.
        /// @param simWaveLength The wave length, in timeseries points, for the
        /// simulated data. See read().
        /// @param sim4bytes Create 4 byte instead of 2 byte integers when
        /// in simulation mode. This simulates the output of the coherent integrator.
        /// @param internalClock Set true if the internal clock should be
        /// used instead of an external clock source.
        p7142Dn(p7142 * myP7142Ptr,
                int chanId, 
                int bypassdivrate = 1,
                int simWaveLength = 5000,
                bool sim4bytes = false,
                bool internalClock = false);
        /// Return the file descriptor for the control device.
        /// @return the file descriptor for the control device.
        int ctrlFd() const;
        /// The P7142 which owns us...
        p7142 & _p7142;
        /// Receiver channel number (0-3)
        int _chanId;
        /// The number of bytes read since the last call to bytesRead()
        long _bytesRead;
        /// The full device name for the downconverter
        std::string _dnName;
        /// The downconverter device file descriptor
        int _dnFd;
        /// The wavelength for simulated data
        unsigned int _simWaveLength;
        /// The counter for keeping track of the current phase during simulation
        unsigned int _angleCount;
        /// True if simulation is supposed to produce 4 byte integers
        bool _sim4bytes;
        /// Mutex for thread safety
        mutable boost::recursive_mutex _mutex;
};

}   // end namespace Pentek

#endif /* P7142DN_H_ */
