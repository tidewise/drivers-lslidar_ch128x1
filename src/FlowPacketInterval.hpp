#ifndef LSLIDAR_CH128X1_FLOWPACKETINTERVAL_HPP
#define LSLIDAR_CH128X1_FLOWPACKETINTERVAL_HPP

namespace lslidar_ch128x1 {
    /**
     * @brief This indicates how the packets are sent by the device
     *
     */
    enum FlowPacketInterval {
        ONE_DIFOP_PACKET_AFTER_FOUR_MSOP_PACKETS = 0,
        ONE_DIFOP_PACKET_PER_SECOND = 1
    };
}

#endif