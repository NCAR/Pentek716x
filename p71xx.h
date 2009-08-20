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
            /// @param simulate Set true if we operate in simulation mode.
			p71xx(std::string devName, bool simulate=false);
			/// Destructor.
			virtual ~p71xx();
			/// @return true if the last operation was succesfull,
			/// false otherwise.
			virtual bool ok();
			/// @return the number of bytes read by a down converter,
			/// since the last call to bytesRead().
			long bytesRead();

		protected:
		    /// Process an ioctl.
		    /// @param fd The file descriptor
		    /// @param ioctlCode The ioctl code.
		    /// @param arg The ioctl argument
		    /// @param errMsg an error message to print if the ioctl fails
		    /// @param doexit If true, call exit(1) if the ioctl returns -1
		    /// @returns The result of the ioctl call.
			int doIoctl(int fd, int ioctlCode, void* arg, std::string errMsg, bool doexit=true);
		    /// Process an ioctl.
		    /// @param fd The file descriptor
		    /// @param ioctlCode The ioctl code.
		    /// @param arg The ioctl argument
		    /// @param errMsg an error message to print if the ioctl fails
		    /// @param doexit If true, call exit(1) if the ioctl returns -1
		    /// @returns The result of the ioctl call.
			int doIoctl(int fd, int ioctlCode, int arg, std::string errMsg, bool doexit=true);
		    /// Process an ioctl.
		    /// @param fd The file descriptor
		    /// @param ioctlCode The ioctl code.
		    /// @param arg The ioctl argument
		    /// @param errMsg an error message to print if the ioctl fails
		    /// @param doexit If true, call exit(1) if the ioctl returns -1
		    /// @returns The result of the ioctl call.
			int doIoctl(int fd, int ioctlCode, double arg, std::string errMsg, bool doexit=true);
			/// Create a random number, with gaussian ditribution
            /// Useful for simulation
            /// @param mean Mean othe result
            /// @param stdDev Standard deviation of the result
            /// @returns The number
            double gauss(double mean, double stdDev);
            /// Set the dma buffer and interrupt buffersize. The
            /// buffersize must be at least 2x the interrupt buffer size.
            /// Perhaps it should be even more?
            /// @param fd file descriptor
            /// @param intbufsize Interrupt buffer size.
            /// @param bufN The driver buffer will be this factor times intbufsize
            /// @return 0 on success, -1 on failure.
            int bufset(int fd, int intbufsize, int bufN);
            /// Indicated the success of the last operation.
            bool _ok;
            /// The root device name
            std::string _devName;
            /// The ctrl device name
            std::string _devCtrl;
	        /// file descriptor for ctrl function
	        int _ctrlFd;
            /// set true if in simulation mode
            bool _simulate;
            /// The number of bytes read by a down converter, since the
            /// last call to bytesRead()
            long _bytesRead;
	};
}

#endif /*P71XX_H_*/
