#ifndef P7140H_
#define P7140H_

#include "p71xx.h"
#include "ptkdrv.h"
#include "ptkddr.h"
#include <string>

namespace Pentek {

	/// Base class for a p7142 digital transceiver card.
	class p7140: public p71xx {

		public:
			/// Constructor,
			/// @param devName The top level device name (e.g.
			/// /dev/pentek/p7140/0. Other device names, such
			/// as ctrl will be constructed as needed.
			/// The ctrl device will be accessed in order to verify
			/// that the card is available. Use isOkay() to verify
			/// that it is.
			p7140(std::string devName);
			/// Destructor.
			virtual ~p7140();

		protected:
	};

	/// A p7140 downconvertor
	class p7140dn: public p7140 {
		public:
			/// Constructor
			/// @param devName The top level device name (e.g.
			/// /dev/pentek/p7140/0.
			/// @param dnName The name of the downconvertor device, e.g. 0B
			/// @decrate The decimation rate
			p7140dn(std::string devName, std::string dnName, int decrate=1);
			/// Destructor
			virtual ~p7140dn();
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
			/// The decimation rate.
			int _decrate;
			/// The down convertor file descriptor
			int _dnFd;

	};
}

#endif /*P7140H_*/
