#ifndef P71XX_H_
#define P71XX_H_

#include "ptkdrv.h"
#include "ptkddr.h"
#include "math.h"
#include <string>

#include <boost/thread/recursive_mutex.hpp>

namespace Pentek {

	/// Base class for a p7142 digital transceiver card.
	class p71xx {

		public:
			/// Constructor,
			/// @param devName The top level device name (e.g., 
            /// /dev/pentek/p7140/0. Other device names, such as ctrl will be 
            /// constructed as needed. The ctrl device will be opened and will 
            /// be available via the _ctrlFd file descriptor. Use ok() to 
            /// verify successful construction.
            /// @param simulate Set true if we operate in simulation mode.
			p71xx(std::string devName, bool simulate=false);
			/// Destructor.
			virtual ~p71xx();
			/// @return true if the last operation was successful,
			/// false otherwise.
			virtual bool ok() const;
			/// Return true iff we're in simulation mode.
			/// @return true iff we're in simulation mode.
			bool isSimulating() const { return _simulate; }

		protected:
            /// Return the file descriptor for the control device.
            /// @return the file descriptor for the control device.
            int ctrlFd() { return _ctrlFd; }
            /// Process an ioctl.
		    /// @param fd The file descriptor
		    /// @param ioctlCode The ioctl code.
		    /// @param arg The ioctl argument
		    /// @param errMsg an error message to print if the ioctl fails
		    /// @param doexit If true, call exit(1) if the ioctl returns -1
		    /// @return The result of the ioctl call.
			static int doIoctl(int fd, int ioctlCode, void* arg, std::string errMsg, bool doexit=true);
		    /// Process an ioctl.
		    /// @param fd The file descriptor
		    /// @param ioctlCode The ioctl code.
		    /// @param arg The ioctl argument
		    /// @param errMsg an error message to print if the ioctl fails
		    /// @param doexit If true, call exit(1) if the ioctl returns -1
		    /// @return The result of the ioctl call.
			static int doIoctl(int fd, int ioctlCode, int arg, std::string errMsg, bool doexit=true);
		    /// Process an ioctl.
		    /// @param fd The file descriptor
		    /// @param ioctlCode The ioctl code.
		    /// @param arg The ioctl argument
		    /// @param errMsg an error message to print if the ioctl fails
		    /// @param doexit If true, call exit(1) if the ioctl returns -1
		    /// @return The result of the ioctl call.
			static int doIoctl(int fd, int ioctlCode, double arg, std::string errMsg, bool doexit=true);
			/// Create a random number, with Gaussian distribution about a 
			/// selected mean and with selected standard deviation.
            /// Useful for simulation
            /// @param[in] mean mean value of the Gaussian distribution
            /// @param[in] stdDev standard deviation of the Gaussian distribution
            /// @return the generated random number
            static double gauss(double mean, double stdDev);
            /// Set the dma buffer and interrupt buffersize. The
            /// buffersize must be at least 2x the interrupt buffer size.
            /// Perhaps it should be even more?
            /// @param fd file descriptor
            /// @param intbufsize Interrupt buffer size.
            /// @param bufN The driver buffer will be this factor times intbufsize
            /// @return 0 on success, -1 on failure.
            static int bufset(int fd, int intbufsize, int bufN);
            /// Indicated the success of the last operation.
            bool _ok;
            /// The root device name
            std::string _devName;
            /// The ctrl device name
            std::string _devCtrl;
	        /// file descriptor for the ctrl device
	        int _ctrlFd;
            /// set true if in simulation mode
            bool _simulate;
            /// recursive mutex which provides us thread safety.
            mutable boost::recursive_mutex _mutex;
	};
}

#endif /*P71XX_H_*/
