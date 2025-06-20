#ifndef LSLIDAR_CH128X1_DRIVER_HPP
#define LSLIDAR_CH128X1_DRIVER_HPP

#include "iodrivers_base/Driver.hpp"
#include "lslidar_ch128x1/Protocol.hpp"
#include <memory>

namespace lslidar_ch128x1 {
    /**
     *
     */
    class Driver : public iodrivers_base::Driver {
    public:
        int extractPacket(uint8_t const* buffer, size_t buffer_size) const;
        Driver();
        static const int PACKET_SIZE = 1206;
        static const int INTERNAL_BUFFER_SIZE = PACKET_SIZE * 4;
        lslidar_ch128x1::Protocol protocol;
        std::optional<base::samples::Pointcloud> read();
        std::unique_ptr<Driver> m_driver;
    };
}

#endif // LSLIDAR_CH128X1_DRIVER_HPP