#include <iomanip>
#include <iostream>
#include <string>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdio.h>
#include <math.h>
#include <sched.h>
#include <sys/timeb.h>

#include "p7142hcr.h"

#define BASICSIZE   1024

using namespace std;

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

	if (argc < 5) {
		std::cout << "usage: " << argv[0]
				<< " <device root> <channel number (0-3)> <bypass decimation rate (1-4096)> <buffer factor> [<output file>]\n";
		exit(1);
	}

	std::string devRoot = argv[1];
	int chanId = atoi(argv[2]);
	int bypdiv = atoi(argv[3]);
	int bufferSize = BASICSIZE * atoi(argv[4]);
	std::string outFile;
	if (argc > 5)
		 outFile = argv[5];

	std::cout << "read buffer size is " << bufferSize << std::endl;

	char* buf = new char[bufferSize];

	// create the output file
	int outFd = -1;
	if (outFile.size() > 0)
		outFd = createOutputFile(outFd, outFile);

	// try to change scheduling to real-time
	makeRealTime();

	// default configuration
	int gates = 500;
	int delay = 0;
	int prt = 2000; // 10 MHz counts
	int prt2 = prt; // no staggered prt
	int pulsewidth = 10; // 10 MHz counts
	bool stgr_prt = false;
	std::string gaussianFile = "";
	std::string kaiserFile = "";

	// create the downconvertor/// @todo The downconverter type needs to be specified in a command line switch
	Pentek::p7142hcrdn downConverter(devRoot, chanId, gates, 256, 1, delay, prt, prt2,
			pulsewidth, stgr_prt, false, gaussianFile, kaiserFile, Pentek::p7142hcrdn::DDC8DECIMATE, bypdiv);

	if (!downConverter.ok()) {
		std::cerr << "cannot access " << downConverter.dnName() << "\n";
		exit(1);
	}

	// start the loop
	int loopCount = 0;
	double total = 0;

	double startTime = nowTime();

	int lastMb = 0;

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
			int mb = (int) (total / 1.0e8);
			if (mb > lastMb) {
				lastMb = mb;
				double elapsed = nowTime() - startTime;
				double bw = (total / elapsed) / 1.0e6;

				int overruns = downConverter.overUnderCount();

				std::cout << "total " << std::setw(5) << mb * 100
						<< " MB,  BW " << std::setprecision(4) << std::setw(5)
						<< bw << " MB/s, overruns: " << overruns << "\n";
			}

		}

		// if writing a file, only allow 2 GB collected.
		if (total > 2.0e9 && outFd != -1)
			break;
	}
}

