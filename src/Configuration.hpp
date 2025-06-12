#ifndef LSLIDAR_CH128X1_CONFIGURATION_HPP
#define LSLIDAR_CH128X1_CONFIGURATION_HPP

#include "lslidar_ch128x1/FlowPacketInterval.hpp"
#include "lslidar_ch128x1/TimeServiceMode.hpp"
#include "lslidar_ch128x1/TimeUTC.hpp"
#include <base/Angle.hpp>
#include <base/Float.hpp>
#include <gps_base/BaseTypes.hpp>

namespace lslidar_ch128x1 {
    /**
     *
     */
    struct Configuration {
        /**
         * @brief The lidar IP
         * Eg. "192.168.88.1"
         */
        std::string lidar_ip;
        /**
         * @brief The computer IP
         * Eg. "192.168.88.1"
         */
        std::string computer_ip;
        uint16_t data_port;
        uint16_t device_port;
        /**
         * @brief true - indicates that the lidar is stationary
         * false - indicates that the lidar is rotating
         */
        bool lidar_stationary;
        /**
         * @brief This indicates how the packets are sent
         */
        FlowPacketInterval flow_packet_interval;
        TimeServiceMode clock_source;
        /**
         * @brief true - indicates that the lidar is in stationary mode
         * false - otherwise
         */
        bool standby_mode;
        /**
         * @brief true - indicates that the lidar's motor is locked at a certain phase
         * lock angle
         * false - otherwise
         */
        bool phase_lock_enabled;
        /**
         * @brief The lidar's motor is locked at this angle, if the phase lock is enabled
         *
         */
        base::Angle phase_lock_angle;
        TimeUTC time_utc;
        /**
         * @brief The lidar's motor speed
         */
        uint16_t motor_speed = 0;
        /**
         * @brief The lidar's mac address
         */
        std::string mac_address;
        gps_base::Position gps_position;
    };
}

#endif