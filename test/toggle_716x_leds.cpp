#include <iomanip>
#include <iostream>
#include <string>
#include <boost/program_options.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <csignal>
#include <logx/Logging.h>
#include "p716x.h"

using namespace std;
using namespace boost::posix_time;
namespace po = boost::program_options;

LOGGING("toggleP716xLEDs")

double _runTimeSecs = 20.0;
double _waitSecs = 0.5;  ///< Wait between ids (secs)

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
    ("runTimeSecs", po::value<double>(&_runTimeSecs),
     "Program run time (secs) (default 20)")
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
    PTK716X_DeviceClose(hDev);
  }
  
  DWORD dwStatus = PTK716X_LibUninit();
  if (PTK716X_STATUS_OK != dwStatus) {
    puts ("Error: PTK716x library un-init failed");
  }
    
  /* display message */
  switch (code) {
    case  0: puts ("Done");                          break;
    case  1: puts ("Error: PTK716x library init failed");        break;
    case  2: puts ("Error: 716x device not found");              break;
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

  DWORD dwStatus = PTK716X_LibInit();
  if (PTK716X_STATUS_OK != dwStatus) {
    return (exitHandler (1, NULL));
  }
  
  // Find and open the next PTK714x device
  // user will be asked to pick the device num
  
  DWORD BAR0Base;
  DWORD BAR2Base;
  DWORD BAR4Base;
  DWORD slot = -1; // forces user interaction if more than 1 pentek
  void *hDev = PTK716X_DeviceFindAndOpen(&slot, &BAR0Base, &BAR2Base, 
                                         &BAR4Base, 71620);
  if (hDev == NULL) {
    return (exitHandler (2, hDev));
  }
  
  // Initialize the 716x register address table
  
  P716x_REG_ADDR p716xRegs;  /* 716x register table */
  P716x_BOARD_RESOURCE boardResource;
  // Initialize 716x register address tables
  P716xInitRegAddr(BAR0Base, &p716xRegs, &boardResource, P71620_MODULE_ID);

  // Reset board registers to power-on default states
  P716xResetRegs(&p716xRegs);
  
  // Load parameter tables with default values
  P716x_GLOBAL_PARAMS globalParams;
  P716xSetGlobalDefaults(&boardResource, &globalParams);

  // check if module is a 716x

  P716x_MODULE_INFO moduleInfo;
  int result = P716xIdentifyBoard(&moduleInfo, &p716xRegs, P71620_MODULE_ID);
  if (result != P716x_TRUE) {
    cerr << "Pentek card " << slot + 1
         << " is not a 71620" << endl;
    cerr << "Expected 0x" << hex << P71620_MODULE_ID
         << ", and got 0x" << moduleInfo.boardId << dec << endl;
    return -1;
  }

  // print status

  cerr << "Found Pentek 716x device";
  cerr << "  device addr: " << hex << hDev << endl;
  cerr << "  slot: " << dec << slot << endl;
  cerr << "  BAR0Base: " << hex << (void *)BAR0Base;
  cerr << "  BAR2Base: " << hex << (void *)BAR2Base;
  cerr << dec;
  cerr << endl;

//  // get master bus controls
//
//  uint32_t masterBusAControl =
//          P716xGetSBusCtrl1GateSyncMaster(p716xRegs.syncBusControl1, 
//                                          P716x_SYNC_BUS_A);
//  uint32_t masterBusBControl =
//          P716xGetSBusCtrl1GateSyncMaster(p716xRegs.syncBusControl1, 
//                                          P716x_SYNC_BUS_B);
//
//  cerr << "Initial state:" << endl;
//  cerr << "  bus A gate/sync master: " << hex << masterBusAControl << endl;
//  cerr << "  bus B gate/sync master: " << hex << masterBusBControl << endl;
//  cerr << dec;

  // Toggle user LED every _waitSecs for _runTimeSecs

  for (int i = 0; (i * _waitSecs) < _runTimeSecs; i++) {
      // alternate user LED on/off
      uint32_t ledState = (i % 2) ? P716x_USER_LED_ON : P716x_USER_LED_OFF;
      P716xSetUserLedState(p716xRegs.userLED, ledState);
      usleep(int(1.0e6 * _waitSecs));
  }

  // set user LED on at exit
    
  cerr << "--->> Setting all on" << endl;
    
  P716xSetUserLedState(p716xRegs.userLED, P716x_USER_LED_ON);

  // return

  return (exitHandler (0, hDev));
  
}

