#ifndef P71XX_H_
#define P71XX_H_

#include "ptkdrv.h"
#include "ptkddr.h"
#include "math.h"
#include <string>

namespace Pentek {
	
	/// Base class for a p7142 digital transceiver card.
	class p71xx {
		
		public:
			/// Constructor,
			/// @param devName The top level device name (e.g. 
			/// /dev/pentek/p7140/0. Other device names, such
			/// as ctrl will be constructed as needed.
			/// The ctrl device will be accessed in order to verify
			/// that the card is available. Use isOkay() to verify
			/// that it is.
			p71xx(std::string devName, bool simulate=false);
			/// Destructor.
			virtual ~p71xx();
			/// @return true if the last operation was succesfull,
			/// false otherwise.
			virtual bool ok();
			
		protected:
			/// Create a random number, with gaussian ditribution
			/// Useful for simulation
			/// @param mean Mean othe result
			/// @param stdDev Standard deviation of the result
			/// @returns The number
			double gauss(double mean, double stdDev);	
			/// Indicated the success of the last operation.
			bool _ok;
			/// The root device name
			std::string _devName;
			/// The ctrl device name
			std::string _devCtrl;
		    /// set true if in simulation mode
		    bool _simulate;
	};
}

#endif /*P71XX_H_*/
