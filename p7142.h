#ifndef P7142_H_
#define P7142_H_

#include "p71xx.h"
#include "ptkdrv.h"
#include "ptkddr.h"
#include <string>

namespace Pentek {

	/// Base class for a p7142 digital transceiver card.
	class p7142: public p71xx {

		public:
			/// Constructor,
			/// @param devName The top level device name (e.g.
			/// /dev/pentek/p7140/0. Other device names, such
			/// as ctrl will be constructed as needed.
			/// The ctrl device will be accessed in order to verify
			/// that the card is available. Use isOkay() to verify
			/// that it is.
			/// @param simulate Set true for simulation mode.
			p7142(std::string devName, bool simulate=false);
			/// Destructor.
			virtual ~p7142();

		protected:
	};

	/// A p7142 downconvertor
	class p7142dn: public p7142 {
		public:
			/// Constructor
			/// @param devName The top level device name (e.g.
			/// /dev/pentek/p7140/0.
			/// @param dnName The name of the downconvertor device, e.g. 0B
			/// @bypassdivrate The byopass divider (decimation) rate
			/// @param simulate Set true if we operate in simulation mode.
			/// @param simPauseMS The number of milliseconds to wait before returning
			/// simulated data when calling read();
			p7142dn(std::string devName, std::string dnName, int bypassdivrate=1,
			bool simulate=false, int simPauseMS=100);
			/// Destructor
			virtual ~p7142dn();
			/// Read bytes.
			/// @param buf read bytes into this buffer
			/// @param bufsize The number of bytes tor read.
			/// @return The actual number of bytes read
			virtual int read(char* buf, int bufsize);
			/// @return The number of overrun, or underrun samples. Return
			/// -1 if unable to get this information, and set _ok to false.
			/// Clear the counter as well.
			virtual int overUnderCount();

		protected:
			/// The down convertor device name
			std::string _dnName;
			/// The bypass divider decimation rate.
			int _bypdiv;
			/// The down convertor file descriptor
			int _dnFd;
			/// The number of milliseconds to wait before returning
			/// simulated data when calling read();
			int _simPauseMS;
	};

	/// A p7142 upconvertor
	class p7142up: public p7142 {
		public:
			/// Constructor
			/// @param devName The top level device name (e.g.
			/// /dev/pentek/p7140/0.
			/// @param upName The name of the downconvertor device, e.g. 0C
			/// @param sampleclockHz The DAC sample clock in Hz
			/// @param ncofreqHz The NCO frequency in Hz
			/// @param simulate Set true if we operate in simulation mode.
			p7142up(std::string devName, std::string upName,
				double sampleClockHz, double ncoFreqHz, char mode, bool simulate=false);
			/// Destructor
			virtual ~p7142up();
			/// Write the baseband signal to ram.
			/// @param buf The I and Q signal values
			/// @param n The number of values.
			virtual void write(short* data, int n);
			/// Start the DAC output
			virtual void startDAC();
			/// Stop the DAC output
			virtual void stopDAC();

		protected:
			/// Calculate the NCO frequency control bytes. It is dependent upon the
			/// the mode (_interp) and the sample clock rate.
			/// @param fNCO The desired NCO frequency in Hz
			/// @param fDAC The DAC clock rate
			/// @param nco_freq_0 Byte 0 of the nco frequency parameter is returned here
			/// @param nco_freq_1 Byte 1 of the nco frequency parameter is returned here
			/// @param nco_freq_2 Byte 2 of the nco frequency parameter is returned here
			/// @param nco_freq_3 Byte 3 of the nco frequency parameter is returned here
			void ncoConfig(double fNCO, double fDAC, char& nco_freq_0, char& nco_freq_0, char& nco_freq_0, char& nco_freq_3);
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
	};
}

#endif /*P7142_H_*/
