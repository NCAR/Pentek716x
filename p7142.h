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
			p7142(std::string devName);
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
			p7142dn(std::string devName, std::string dnName, int bypassdivrate=1);
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
			/// The down convertor file descriptor
			int _dnFd;
			/// The bypass divider decimation rate.
			int _bypdiv;

	};
}

#endif /*P7142_H_*/
