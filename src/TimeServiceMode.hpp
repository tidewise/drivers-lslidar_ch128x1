#ifndef LSLIDAR_CH128X1_TIMESERVICEMODE_HPP
#define LSLIDAR_CH128X1_TIMESERVICEMODE_HPP

namespace lslidar_ch128x1 {
    /**
     * @brief The time synchronization. It could be done with GPS or PTP (Precision
     * Time Protocol)
     */
    enum TimeServiceMode {
        GPS = 0,
        PTP = 1
    };
}

#endif