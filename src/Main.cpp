#include <lslidar_ch128x1/Driver.hpp>
#include <lslidar_ch128x1/Protocol.hpp>
#include <iomanip>
#include <iostream>

using namespace std;
using namespace lslidar_ch128x1;

int usage()
{
    cerr << "Usage: "
         << "lslidar_ch128x1_ctl URI PERIOD \n"
         << "URI is a valid iodrivers_base URI, e.g. serial:///dev/ttyUSB0:19200\n"
         << "PERIOD is time interval in seconds between two outputs, the default value "
            "is 0.1 seconds\n"
         << flush;
    return 0;
}
int main(int argc, char const* argv[])
{
    if (argc < 2) {
        cerr << "not enough arguments" << endl;
        return usage();
    }
    string uri(argv[1]);
    Driver driver;
    driver.setReadTimeout(base::Time::fromMilliseconds(1000));
    driver.setWriteTimeout(base::Time::fromMilliseconds(1000));
    driver.openURI(uri);
    int poll_period_usec = 100000;
    if (argc >= 3) {
        poll_period_usec = atof(argv[2]) * 1000000;
    }
    while (true) {
        driver.read();
        usleep(poll_period_usec);
    }
}
