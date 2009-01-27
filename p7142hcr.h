/*
 * p7172hcr.h
 *
 *  Created on: Jan 26, 2009
 *      Author: hcr
 */

#ifndef P7172HCR_H_
#define P7172HCR_H_

#include "p7142.h"

namespace Pentek {

	/// A p7142 downconvertor for use with the hcr firmware
	class p7142hcrdn: public p7142dn {
		public:
			/// Constructor
			/// @param devName The top level device name (e.g.
			/// /dev/pentek/p7140/0.
			/// @param dnName The name of the downconvertor device, e.g. 0B
			/// @param gates The number of gates
			/// @bypassdivrate The byopass divider (decimation) rate
			p7142hcrdn(std::string devName, std::string dnName, int gates, int bypassdivrate=1);
			/// Destructor
			virtual ~p7142hcrdn();

		protected:
			/// Configure the p7142hcrdn
			/// @return True if the configuration was succesful
			bool config();
			/// number of gates
			int _gates;
	};

}


#endif /* P7172HCR_H_ */
