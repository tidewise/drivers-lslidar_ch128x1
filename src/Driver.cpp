#include <lslidar_ch128x1/Driver.hpp>

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

std::optional<base::samples::Pointcloud> Driver::read()
{
    uint8_t buffer[INTERNAL_BUFFER_SIZE];
    readPacket(buffer, INTERNAL_BUFFER_SIZE);
    return protocol.handleData(buffer);
}