#include <iostream>
#include <lslidar_ch128x1/Driver.hpp>
#include <lslidar_ch128x1/Protocol.hpp>

using namespace std;
using namespace lslidar_ch128x1;

int usage()
{
    cerr << "Usage: "
         << "lslidar_ch128x1_ctl URI \n"
         << "URI is a valid iodrivers_base URI, e.g. udp://192.168.1.200:2369:2368\n"
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
    while (true) {
        auto point_cloud = driver.read();
        if (point_cloud) {
            cout << "Point cloud size = " << point_cloud->points.size() << endl;
        }
    }
}
