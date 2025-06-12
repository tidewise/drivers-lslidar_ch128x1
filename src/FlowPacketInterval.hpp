#ifndef LSLIDAR_CH128X1_FLOWPACKETINTERVAL_HPP
#define LSLIDAR_CH128X1_FLOWPACKETINTERVAL_HPP

namespace lslidar_ch128x1 {
    /**
     * @brief This indicates how the packets are sent by the device
     * The data packet is the MSOP and the device packet is the DIFOP
     */
    enum FlowPacketInterval {
        ONE_DEVICE_PACKET_AFTER_FOUR_DATA_PACKETS = 0,
        ONE_PACKET_PER_SECOND = 1
    };
}

#endif