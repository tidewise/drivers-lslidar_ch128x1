#include <lslidar_ch128x1/Driver.hpp>
#include "lslidar_ch128x1/Protocol.hpp"
#include <iostream>

using namespace lslidar_ch128x1;
using namespace std;

Driver::Driver()
    : iodrivers_base::Driver::Driver(INTERNAL_BUFFER_SIZE)
{
    protocol = Protocol();
}

int Driver::extractPacket(uint8_t const* buffer, size_t buffer_size) const
{
    if (buffer_size < PACKET_SIZE) {
        return 0;
    }
    return buffer_size;
}

void Driver::read()
{
    uint8_t buffer[INTERNAL_BUFFER_SIZE];
    int packet_size = readPacket(buffer, INTERNAL_BUFFER_SIZE);
    cout << "packet_size: " << packet_size << endl;
    protocol.handleSingleEcho(buffer);
    // parsePacket is a method that is specific to this driver.
    // It gets a packet, extracts the information from it and
    // updates the driver internal data structures
}