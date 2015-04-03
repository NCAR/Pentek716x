#include <iomanip>
#include <iostream>
#include <string>
#include <boost/program_options.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <csignal>
#include <p716x_sd3c.h>
#include <logx/Logging.h>

LOGGING("sd3cInfo");

using namespace std;
using namespace boost::posix_time;
namespace po = boost::program_options;

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
            ("help", "Print the help message and exit")
            ;

    po::variables_map vm;
    po::command_line_parser parser(argc, argv);
    po::positional_options_description pd;
    po::store(parser.options(descripts).positional(pd).run(), vm);
    po::notify(vm);

    if (vm.count("help")) {
        cout << "Print the SD3C DDC type and bitstream revision number " <<
                "installed on a Pentek 716x card";
        cout << "Usage: " << argv[0];
        cout << descripts << endl;
        exit(0);
    }

}

///////////////////////////////////////////////////////////
void
handle_signal(int signal) {
    cerr << "Caught signal " << signal << endl;
}

///////////////////////////////////////////////////////////
int
main(int argc, char** argv)
{

    // Set up logx early. It gets and strips the arguments it cares about
    // from argv.

    logx::ParseLogArgs(argc, argv);


    // parse the command line options, substituting for config params.
    parseOptions(argc, argv);

    // set to ignore SIGPIPE errors which occur when sockets
    // are broken between client and server
    signal(SIGPIPE, SIG_IGN);
    
    // Catch SIGINT, which may be raised by the p716x_sd3c constructor
    signal(SIGINT, handle_signal);

    // Initialize the library

    DWORD dwStatus = PTK716X_LibInit();
    if (PTK716X_STATUS_OK != dwStatus) {
        cerr << "Failed to initialize PTK716X library!" << endl;
        return(1);
    }

    // Find and open the PTK716x device
    // user will be asked to pick the device num

    Pentek::p716x_sd3c sd3cCard(100.0e6, true, true, false, 10.0, 
                                Pentek::p716x_sd3c::DDC10DECIMATE,
                                0.0, 1.0e-6, 1000, 0, false, 100, 1, false,
                                false, false, 0);
    if (sd3cCard.ok()) {
        cout << "Card has SD3C " << sd3cCard.ddcTypeName() << 
                " bitstream, revision " << sd3cCard.sd3cRev() << endl;
    } else {
        cerr << "Unable to instatiate p716x_sd3c!" << endl;
    }

    return(0);
}

