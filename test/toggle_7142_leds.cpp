#include <iomanip>
#include <iostream>
#include <string>
#include <boost/program_options.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <csignal>
#include <logx/Logging.h>
#include "p7142.h"

using namespace std;
using namespace boost::posix_time;
namespace po = boost::program_options;

LOGGING("toggleP7142LEDs")

int _nToggles = 5; ///< number of times leds are toggled
double _waitSecs = 1.0;  ///< Wait between ids (secs)

//////////////////////////////////////////////////////////////////////
//
/// Parse the command line options, and also set some options
/// that are not specified on the command line.
/// @return The runtime options that can be passed to the
/// threads that interact with the RR314.

void parseOptions(int argc, char** argv)
{
  
  // get the options
  po::options_description descripts("Options");
  descripts.add_options()
    ("help", "Describe options")
    ("nToggles", po::value<int>(&_nToggles),
     "No. of times lights are toggled (default 5)")
    ("waitSecs", po::value<double>(&_waitSecs), 
     "Wait between toggles (secs) (default 0.5)")
    ;

  po::variables_map vm;
  po::command_line_parser parser(argc, argv);
  po::positional_options_description pd;
  po::store(parser.options(descripts).positional(pd).run(), vm);
  po::notify(vm);
  
  if (vm.count("help")) {
    cout << "Usage: " << argv[0] << 
      " [OPTION]..." << endl;
    cout << descripts << endl;
    exit(0);
  }
  
}

/**************************************************************************
 Function: exitHandler
 Description:  Routine to set LEDs and output an appropriate message at
               program exit.  Provided to make example code cleaner.
 Inputs:       code - exit code
 Return:       exit code 
**************************************************************************/

int exitHandler(int code, void *hDev)

{

  /* Perform necessary cleanup before exiting the program */
  if (hDev) {
    PTK714X_DeviceClose(hDev);
  }
  
  DWORD dwStatus = PTK714X_LibUninit();
  if (PTK714X_STATUS_OK != dwStatus) {
    puts ("Error: PTK7142 library un-init failed");
  }
    
  /* display message */
  switch (code) {
    case  0: puts ("Done");                          break;
    case  1: puts ("Error: PTK7142 library init failed");        break;
    case  2: puts ("Error: 7142 device not found");              break;
  }
  
  return (code);

}

///////////////////////////////////////////////////////////
int
main(int argc, char** argv)
{

  // Let logx get and strip out its arguments
  logx::ParseLogArgs(argc, argv);
  
  // parse the command line options, substituting for config params.
  parseOptions(argc, argv);

  // set to ignore SIGPIPE errors which occur when sockets
  // are broken between client and server
  signal(SIGPIPE, SIG_IGN);
  
  // Initialize the library

  DWORD dwStatus = PTK714X_LibInit();
  if (PTK714X_STATUS_OK != dwStatus) {
    return (exitHandler (1, NULL));
  }
  
  // Find and open the next PTK714X device
  // user will be asked to pick the device num
  
  DWORD BAR0Base;
  DWORD BAR2Base;
  DWORD slot = -1; // forces user interaction if more than 1 pentek
  void *hDev = PTK714X_DeviceFindAndOpen(&slot, &BAR0Base, &BAR2Base);
  if (hDev == NULL) {
    return (exitHandler (2, hDev));
  }
  
  // Initialize the 7142 register address table
  
  P7142_REG_ADDR p7142Regs;  /* 7142 register table */
  P7142InitRegAddr(BAR0Base, BAR2Base, &p7142Regs);

  // check if module is a 7142

  unsigned int moduleId;
  P7142_GET_MODULE_ID(p7142Regs.BAR2RegAddr.idReadout, moduleId);
  if (moduleId != P7142_MODULE_ID) {
    cerr << "Pentek card " << slot + 1
         << " is not a 7142!" << endl;
    cerr << "Expected 0x" << hex << P7142_MODULE_ID << 
      ", and got 0x" << moduleId << dec << endl;
    return -1;
  }

  // print status

  cerr << "Found Pentek 7142 device";
  cerr << "  device addr: " << hex << hDev << endl;
  cerr << "  slot: " << dec << slot << endl;
  cerr << "  BAR0Base: " << hex << (void *)BAR0Base;
  cerr << "  BAR2Base: " << hex << (void *)BAR2Base;
  cerr << dec;
  cerr << endl;

  // get master bus controls

  uint32_t masterBusAControl;
  P7142_REG_READ(BAR2Base + P7142_MASTER_BUS_A_CONTROL, masterBusAControl);
  
  uint32_t masterBusBControl;
  P7142_REG_READ(BAR2Base + P7142_MASTER_BUS_B_CONTROL, masterBusBControl);

  cerr << "Initial state:" << endl;
  cerr << "  masterBusAControl: " << hex << masterBusAControl << endl;
  cerr << "  masterBusBControl: " << hex << masterBusBControl << endl;
  cerr << dec;

  // toggle master control on and off, to toggle LEDs

  uint32_t bothOnA = masterBusAControl | 0x00000003;
  uint32_t bothOnB = masterBusBControl | 0x00000003;
  
  uint32_t masterOnA = bothOnA | 0x00000001;
  uint32_t masterOnB = bothOnB | 0x00000001;

  uint32_t masterOffA = bothOnA & 0xfffffffe;
  uint32_t masterOffB = bothOnB & 0xfffffffe;

  uint32_t termOnA = bothOnA | 0x00000002;
  uint32_t termOnB = bothOnB | 0x00000002;
  
  uint32_t termOffA = bothOnA & 0xfffffffd;
  uint32_t termOffB = bothOnB & 0xfffffffd;

  useconds_t sleepTime = (int) (_waitSecs * 1.0e6);

  cerr << "--->> Setting all on" << endl;
    
  P7142_REG_WRITE(BAR2Base + P7142_MASTER_BUS_A_CONTROL, bothOnA);
  P7142_REG_WRITE(BAR2Base + P7142_MASTER_BUS_B_CONTROL, bothOnB);

  for (int ii = 0; ii < _nToggles; ii++) {

    cerr << "--->> Toggling master off" << endl;
    
    P7142_REG_WRITE(BAR2Base + P7142_MASTER_BUS_A_CONTROL, masterOffA);
    P7142_REG_WRITE(BAR2Base + P7142_MASTER_BUS_B_CONTROL, masterOffB);

    usleep(sleepTime);

    cerr << "--->> Toggling master on" << endl;
    
    P7142_REG_WRITE(BAR2Base + P7142_MASTER_BUS_A_CONTROL, masterOnA);
    P7142_REG_WRITE(BAR2Base + P7142_MASTER_BUS_B_CONTROL, masterOnB);
    
    usleep(sleepTime);
    
    cerr << "--->> Toggling term off" << endl;
    
    P7142_REG_WRITE(BAR2Base + P7142_MASTER_BUS_A_CONTROL, termOffA);
    P7142_REG_WRITE(BAR2Base + P7142_MASTER_BUS_B_CONTROL, termOffB);

    usleep(sleepTime);

    cerr << "--->> Toggling term on" << endl;
    
    P7142_REG_WRITE(BAR2Base + P7142_MASTER_BUS_A_CONTROL, termOnA);
    P7142_REG_WRITE(BAR2Base + P7142_MASTER_BUS_B_CONTROL, termOnB);
    
    usleep(sleepTime);
    
  }

  // reset to the initial state
    
  // cerr << "--->> Resetting to initial state" << endl;
  // P7142_REG_WRITE(BAR2Base + P7142_MASTER_BUS_A_CONTROL, masterBusAControl);
  // P7142_REG_WRITE(BAR2Base + P7142_MASTER_BUS_B_CONTROL, masterBusBControl);

  // set all on to exit
    
  cerr << "--->> Setting all on" << endl;
    
  P7142_REG_WRITE(BAR2Base + P7142_MASTER_BUS_A_CONTROL, bothOnA);
  P7142_REG_WRITE(BAR2Base + P7142_MASTER_BUS_B_CONTROL, bothOnB);

  // return

  return (exitHandler (0, hDev));
  
}

