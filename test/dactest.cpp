#include <iomanip>
#include <iostream>
#include <string>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdio.h>
#include <math.h>
#include <sched.h>
#include <sys/timeb.h>
#include <stdio.h>
#include <signal.h>
#include <unistd.h>

#include "p7142sd3c.h"

#define BASICSIZE   1024

using namespace std;

bool stop = false;
///////////////////////////////////////////////////////////
void handler( int signal ) {
	stop = true;
}

///////////////////////////////////////////////////////////
int main(int argc, char** argv) {

	if (argc < 6) {
		std::cout << "usage: " << argv[0] << "\n" << "\
    <device root (e.g. /dev/pentek/0/)>\n\
    <up converter name (e.g. 0C)>\n\
    <sample rate Hz>\n\
    <nco frequency Hz>\n\
    <cm_mode 0-15 (The DAC CONFIG2 coarse mixer mode, see DAC5687 data sheet.)>\n";
		exit(1);
	}

	std::string devRoot = argv[1];
	std::string upName  = argv[2];
	double sampleRate   = atof(argv[3]);
	double ncoFreq      = atof(argv[4]);
	char mode           = atoi(argv[5]);

	// create the p7142 and its upconverter
	Pentek::p7142 card(devRoot, false);
	Pentek::p7142Up & upConverter = *card.addUpconverter(upName, sampleRate, 
	        ncoFreq, mode);

	// create the signal
	unsigned int n = 100;
	long IQ[n];
	for (unsigned int i = 0; i < n/4; i++) {
		IQ[i]   = 0x7FFF << 16 | 0x7FFF;
	}
	for (unsigned int i = n/4; i < n/2; i++) {
		IQ[i]   = 0x8000 << 16 | 0x8000;
	}
    for (unsigned int i = n/2; i < n; i++) {
		IQ[i]   = 0;
	}


   for (unsigned int i = 0; i < n; i++) {
		IQ[i]   = 0x7FFF << 16 | 0x7FFF;
	}

	// load mem2
	upConverter.write(IQ, n);

	// start the upconverter

	upConverter.startDAC();

	signal( SIGINT, handler );
	while (1) {
		std::cout << ".";
		std::cout << std::flush;
		sleep(1);
		if (stop)
			break;
	}

	upConverter.stopDAC();
	std::cout << std::endl;
	std::cout << "DAC stopped" << std::endl;

}

