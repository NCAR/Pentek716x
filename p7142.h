#ifndef P7142_H_
#define P7142_H_

#include "p71xx.h"
#include "p7142Dn.h"
#include "p7142Up.h"
#include "ptkdrv.h"
#include "ptkddr.h"
#include "DDCregisters.h"

#include <iostream>
#include <string>
#include <vector>


namespace Pentek {

	/// Base class for a p7142 digital transceiver card.
	class p7142 : public p71xx {

		public:
			/// Constructor.
			/// @param devName The top level device name (e.g.,
			/// /dev/pentek/p7142/0. Use ok() to verify successful construction.
			/// @param simulate Set true for simulation mode.
			p7142(std::string devName, bool simulate=false);
			/// Destructor.
			virtual ~p7142();
            /// A P7142 card has 4 receive channels available.
            static const int P7142_NCHANNELS = 4;
            /// (Suggested) time to sleep after P7142 ioctl calls, in microseconds
            static const int P7142_IOCTLSLEEPUS = 100;
			/// Return the base device name for our P7142 card.
			/// @return the base device name.
			std::string devName() const { return _devName; }
			
            /// Construct and add a downconverter for one of our receiver channels.
            /// @param chanId The channel identifier (used to select /dn/*B)
            /// @param bypassdivrate The bypass divider (decimation) rate
            /// @param simulate Set true if we operate in simulation mode.
            /// @param simWaveLength The wave length, in timeseries points, for the
            /// simulated data. See p7142Dn::read().
            /// @param sim4bytes Create 4 byte instead of 2 byte integers when
            /// in simulation mode. This simulates the output of the coherent 
            /// integrator.
            virtual p7142Dn * addDownconverter(int chanId, int bypassdivrate = 1,
                    int simWavelength = 5000, bool sim4bytes = false);
            
            /// Construct and add an upconverter for our device.
            /// @param upName The name of the downconverter device, e.g. 0C
            /// @param sampleClockHz The DAC sample clock in Hz
            /// @param ncoFreqHz The NCO frequency in Hz
            /// @param mode The DAC CONFIG2 coarse mixer mode (See DAC5687 Data Sheet)
            virtual p7142Up * addUpconverter(std::string upName, 
                    double sampleClockHz, double ncoFreqHz, char mode);
            
            // We make our associated downconverter and upconverter classes 
            // friends so that they have access to _ctrlFd, the doIoctl()
            // methods, etc.
            friend class p7142Dn;
            friend class p7142Up;

		protected:
            /// Add (or replace) a downconverter on our list. If the 
            /// downconverter is associated with a channel for which we already
            /// have a downconverter, the existing downconverter for that
            /// channel will be deleted. This object assumes ownership of the 
            /// incoming downconverter.
            /// @param downconverter the downconverter to be added.
            void _addDownconverter(p7142Dn * downconverter);
            
            /// Add (or replace) our upconverter. If we already have an
            /// upconverter, it will be deleted. This object assumes ownership 
            /// of the incoming upconverter.
            /// @param downconverter the downconverter to be added.
            void _addUpconverter(p7142Up * upconverter);
            
			std::vector<p7142Dn*> _downconverters;
			p7142Up * _upconverter;
	};

} // end namespace Pentek

#endif /*P7142_H_*/
