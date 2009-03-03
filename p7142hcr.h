/*
 * p7172hcr.h
 *
 *  Created on: Jan 26, 2009
 *      Author: hcr
 */

#ifndef P7172HCR_H_
#define P7172HCR_H_

#include "p7142.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <sys/types.h>
#include <sys/stat.h>
#include <pthread.h>

#include <string>
#include <vector>
#include <deque>
#include <map>

#include <stdio.h>
#include <stdlib.h>
#include "FilterSpec.h"
#include "DDCregisters.h"

namespace Pentek {

	/// A p7142 downconvertor for use with the HCR firmware
	class p7142hcrdn: public p7142dn {
		public:
			/// Constructor
			/// @param devName The top level device name (e.g.
			/// /dev/pentek/p7140/0.
			/// @param dnName The name of the downconvertor device, e.g. 0B
			/// @param chanId The channel identifier (should be a zero based small integer)
			/// @param gates The number of gates
			/// @param delay the delay to the first gate in 10 MHz counts
			/// @param prt the radar PRT in 10 MHz counts
			/// @param prt2 the second PRT of a staggered PRT sequence, expressed in 10 MHz counts
			/// @param pulse_width the radar pulse width in 10 MHz counts
			/// @param stgr_prt the staggered PRT flag; 1 = stagger, 0 = uniform
			/// @bypassdivrate The byopass divider (decimation) rate
			/// @param simulate Set true if we operate in simulation mode.
			/// @param simPauseMS The number of milliseconds to wait before returning
			/// simulated data when calling read();
			p7142hcrdn(std::string devName, std::string dnName,
					   int chanId, int gates, int delay, int prt,
					   int prt2, int pulse_width, bool stgr_prt,
					   std::string gaussianFile, std::string kaiserFile,
					   int bypassdivrate=1, bool simulate=false, int simPauseMS=100);
			/// Destructor
			virtual ~p7142hcrdn();

		protected:
			/// Configure the p7142hcrdn
			/// @return True if the configuration was successful

			bool config();

			/// Configure the filters and the decimation value.
	        int filterSetup();

	        /// Program the cofficients for the gaussian and
	        /// kaiser filters.
	        bool loadFilters(FilterSpec& gaussian,FilterSpec& kaiser);

	        /// Configure the timers.
	        /// @return true if successful, false otherwise.
	        bool timerInit();

	        /// Starts the timers.
	         void startInternalTimer();

	         /// Set the time of the first transmit pulse.
	         /// @param startTime The boost::posix_time::ptime of the first transmit
	         ///    pulse.
	         void setXmitStartTime(boost::posix_time::ptime startTime);

	         /// Time of first transmit pulse
	         boost::posix_time::ptime xmitStartTime()
	         {
	             return _xmitStartTime;
	         }

			/// number of gates
			int _gates;
			/// radar PRT in 10 MHz counts
			int _delay;
			/// second PRT of staggered PRT in 10 MHz counts
			int _prt;
			/// delay to first gate in 10 MHz counts
			int _prt2;
			/// radar pulse width in 10 MHz counts
			int _pulse_width;
			/// staggered PRT flag: 1 = stagger, 0 = uniform
			bool _stgr_prt;
	        /// The path to the file containing the gaussian filter definitions.
	        std::string _gaussianFile;
	        /// The path to the file containing the kaiser filter coefficients.
	        std::string _kaiserFile;
	        /// Time of the first xmit pulse.
	         boost::posix_time::ptime _xmitStartTime;
	        /// peak-poke structure pointer
	         ARG_PEEKPOKE _pp;
	        /// file descriptor for ctrl function
	         int _ctrlFd;

	};

}


#endif /* P7172HCR_H_ */
