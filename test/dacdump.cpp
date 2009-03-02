#include <iostream>
#include <string>
#include <fcntl.h>
#include <sys/ioctl.h>
#include "ptkdrv.h"
#include "ptkddr.h"

////////////////////////////////////////////////////////////////////////////////////////
char
getDACreg(int fd, int reg) {

  ARG_PEEKPOKE pp;
  pp.offset = reg;
  pp.page = 0;
  pp.mask = 0;

  int status = ioctl(fd,FIOREGGET,(long)&pp);
  if (status < 0) {
	  perror("FIOREGGET ioctl error");
  }

  return(pp.value);
}

////////////////////////////////////////////////////////////////////////////////////////
void
dumpDACregs(int fd) {
	for (int i = 0; i < 32; i++) {
		// get value
		char val = getDACreg(fd, i);
		std::cout << "DAC register 0x"  << std::hex  << i << std::dec << ":";
		// print binary
		for (int i = 0; i < 8; i++) {
		    char mask = 1 << 7-i;
			std::cout << " ";
			std::cout << ((val & mask)? "1":"0");
		}
		// print hex
		std::cout << "  " << std::hex << (((int)val) & 0xff) << std::dec << "     ";
		std::cout << std::endl;
	}
}

////////////////////////////////////////////////////////////////////////////////////////
int
main(int argc, char** argv) {
	if (argc != 2) {
		std::cerr << "usage: dacdump <device>\n";
		return 1;
	}

	std::string upName(argv[1]);

	// open upconverter
	int upFd = open(upName.c_str(), O_RDONLY);

    if (upFd < 0) {
      std::cerr << "unable to open " << upName << std::endl;
      return 1;
    }

    std::cout << "DAC registers for " << upName << std::endl;

    dumpDACregs(upFd);

}

