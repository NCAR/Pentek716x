#include <iomanip>
#include <iostream>
#include <string>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdio.h>
#include <math.h>
#include <sched.h>
#include <sys/timeb.h>
#include <boost/program_options.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
// For configuration management
#include <QtConfig.h>

#include "p7142sd3c.h"

#define BASICSIZE   1024

using namespace std;
using namespace boost::posix_time;
namespace po = boost::program_options;

std::string _devRoot;            ///< Device root e.g. /dev/pentek/0
int _chanId;                     ///< The channel device number (0-3)
int _gates;                      ///< The number of gates
int _nsum;                       ///< The number of sums
int _tsLength;                   ///< The time series length
double _delay;				     ///< delay in seconds
double _prt;                     ///< prt in seconds
double _prt2=0.0;                ///< prt2 in seconds
double _pulseWidth;              ///< pulsewidth in seconds
bool _stgrPrt = false;           ///< set true for staggered prt
std::string _gaussianFile = "";  ///< gaussian filter coefficient file
std::string _kaiserFile = "";    ///< kaiser filter coefficient file
int _decim;                      ///< Decimation or bypass divider rate
bool _internalClock = false;     ///< set true to use the internal clock, false otherwise
bool _freeRun = false;           ///< If set true, the prf gating of the downconversion is disabled.
int _bufFactor = 10;			 ///< The read buffer size is _bufFactor * BASICSIZE
std::string _outFile;            ///< File name for collecting output data, if desired

//////////////////////////////////////////////////////////////////////
///
/// get parameters that are specified in the configuration file.
/// These can be overriden by command line specifications.
void getConfigParams()
{

	QtConfig config("PentekCapture", "PentekCapture");

	// get parameters
	_devRoot       = config.getString("Device/DeviceRoot",  "/dev/pentek/p7142/0");
	_gates         = config.getInt("Radar/Gates",           400);
	_prt		   = config.getDouble("Radar/PRT", 			0.2); // 5 Hz
	_pulseWidth    = config.getDouble("Radar/PulseWidth", 	0.000001);    // 1 uS
	_nsum          = config.getInt("Radar/Nsum",            1);
	_tsLength      = config.getInt("Radar/TsLength",        256);
	_freeRun       = config.getBool("Radar/FreeRunning",    false);
	_internalClock = config.getBool("InternalClock",        false);
	_bufFactor     = config.getInt("ReadBufferFactor",      1);
	_outFile       = config.getString("CaptureFile",        "");

}

//////////////////////////////////////////////////////////////////////
//
/// Parse the command line options, and also set some options
/// that are not specified on the command line.
/// @return The runtime options that can be passed to the
/// threads that interact with the RR314.
void parseOptions(int argc,
		char** argv)
{

	// get the options
	po::options_description descripts("Options");
	descripts.add_options()
	("help", "Describe options")
	("devRoot", po::value<std::string>(&_devRoot),     "Device root (e.g. /dev/pentek/0)")
	("channel", po::value<int>(&_chanId),              "Channel number (0-3)")
	("gates",  po::value<int>(&_gates),                "Number of gates")
	("prt",  po::value<double>(&_prt),                 "PRT in seconds")
	("pulseWidth",  po::value<double>(&_pulseWidth),   "Pulse width seconds")
	("nsum",  po::value<int>(&_nsum),                  "Number of coherent integrator sums")
	("freeRun",                                        "Free running mode, PRT gating is disabled")
    ("internalClock",                                  "Use the internal clock instead of the front panel clock")
    ("bufferFactor",  po::value<int>(&_bufFactor),     "Read buffer factor")
    ("captureFile", po::value<std::string>(&_outFile), "Capture file name (no capture if not specified)")
			;

	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, descripts), vm);
	po::notify(vm);

	if (vm.count("internalClock"))
	    _internalClock = true;
	if (vm.count("freeRun"))
		_freeRun = true;

	if (vm.count("help")) {
		std::cout << descripts << std::endl;
		exit(1);
	}
}

///////////////////////////////////////////////////////////
void argumentCheck() {

	if (_nsum < 0 || (_nsum > 1 && (_nsum%2 != 0))) {
		std::cerr << "nsum must be greater than 0 and less than 65535. If between 2 and 65535, it must be even." << std::endl;
		exit(1);
	}

	if (_gates < 1 || _gates > 511) {
		std::cerr << "gates must be greater than 0 and less than 512." << std::endl;
		exit(1);
	}

}

///////////////////////////////////////////////////////////
int createOutputFile(int curFd, std::string name) {

	if (curFd > -1)
		close(curFd);

	int fd = open(name.c_str(), O_WRONLY| O_CREAT | O_TRUNC, 0666);

	if (fd < 0) {
		std::cerr << "cannot access " << name << "\n";
		perror("");
		exit(1);
	}

	std::cout << "Capture file named " << name << " will be created." << std::endl
	          << "   It will contain 16 bit binary I and Q values." << std::endl
	          << "   Use the od command to convert to text: \"od -w4 -s -An -v <file name>\"." << std::endl;

	return fd;
}

///////////////////////////////////////////////////////////
void makeRealTime() {

	uid_t id = getuid();

	// don't even try if we are not root.
	if (id != 0) {
		std::cerr << "Not root, unable to change scheduling priority"
				<< std::endl;
		return;
	}

	sched_param sparam;
	sparam.sched_priority = 50;

	if (sched_setscheduler(0, SCHED_RR, &sparam)) {
		std::cerr << "warning, unable to set scheduler parameters: ";
		perror("");
		std::cerr << "\n";
	}
}

///////////////////////////////////////////////////////////
double nowTime() {
	struct timeb timeB;

	ftime(&timeB);

	return timeB.time + timeB.millitm / 1000.0;

}

///////////////////////////////////////////////////////////
int main(int argc, char** argv) {

	// get the configuration parameters from the configuration file
	getConfigParams();

	// parse the command line options, substituting for config params.
	parseOptions(argc, argv);

    // make sure that the specified arguments are compatible
	argumentCheck();

    int bufferSize = BASICSIZE * _bufFactor;
    std::cout << "read buffer size is " << bufferSize << std::endl;

	char* buf = new char[bufferSize];

	// create the output file
	int outFd = -1;
	if (_outFile.size() > 0)
		outFd = createOutputFile(outFd, _outFile);

	// try to change scheduling to real-time
	makeRealTime();

    // Instantiate our p7142 object and create the downconverter on it
	Pentek::p7142sd3c sd3c(_devRoot, false, 0, _pulseWidth, _prt, _prt2, 
	        _stgrPrt, _gates, _nsum, _freeRun);
	Pentek::p7142sd3cDn & downConverter = *sd3c.addDownconverter(_chanId, false, 
	        _tsLength, 0, _pulseWidth, _gaussianFile, _kaiserFile);
	

	sd3c.startFilters();
	sd3c.timersStartStop(true);

	// start the loop
	int loopCount = 0;
	double total = 0;

	double startTime = nowTime();
	double loopTime = startTime;

	while (1) {
		int n = downConverter.read(buf, bufferSize);
		if (n <= 0) {
			std::cerr << "read returned " << n << " ";
			if (n < 0)
				perror("");
			std::cerr << "\n";
		} else {
			if (outFd != -1)
				write(outFd, buf, n);
			total += n;
			loopCount++;
			double mb = (total / 1.0e6);
			if (nowTime() - loopTime >= 10.0) {
				loopTime = nowTime();
				double elapsed = nowTime() - startTime;
				double bw = (total / elapsed) / 1.0e6;

				int overruns = downConverter.overUnderCount();

				std::cout
				<< "total "   << std::setw(5) << mb << " MB,  "
				<< "elapsed " << std::setw(8) << elapsed << " s, "
				<< "BW "      << std::setprecision(4) << std::setw(5) << bw << " MB/s, "
				<< "overruns: " << overruns << "\n";
			}

		}

		// if writing a file, only allow 2 GB collected.
		if (total > 2.0e9 && outFd != -1)
			break;
	}
}

