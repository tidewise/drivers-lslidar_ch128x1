#ifndef LSLIDAR_CH128X1_PROTOCOL_HPP
#define LSLIDAR_CH128X1_PROTOCOL_HPP

#include "lslidar_ch128x1/TimeServiceMode.hpp"
#include "lslidar_ch128x1/UTC_Time.hpp"
#include <base/Float.hpp>
#include <base/samples/Pointcloud.hpp>

namespace lslidar_ch128x1 {
    /**
     *
     */
    class Protocol {
    public:
        base::samples::Pointcloud handleSingleEcho(unsigned char* data);
        Protocol();
        void handleDIFOP(unsigned char* data);

    private:
        double m_sin_theta_1[128];
        double m_sin_theta_2[128];
        double m_cos_theta_1[128];
        double m_cos_theta_2[128];
        base::Point getPoint(uint8_t line_number,
            double horizontal_angle_degree,
            double distance);
        void computeDefaultSinesAndCosines();
        double computeVerticalAngleSin(uint8_t line_number,
            double horizontal_angle_degree);
        UTC_Time m_utc_time;
        TimeServiceMode m_time_service_mode;
        void getOffsetAngle(std::vector<int> const& prism_angles);
    };
    static constexpr double EMISSION_ANGLES_DEGREE[32] = {-17,
        -16,
        -15,
        -14,
        -13,
        -12,
        -11,
        -10,
        -9,
        -8,
        -7,
        -6,
        -5,
        -4.125,
        -4,
        -3.125,
        -3,
        -2.125,
        -2,
        -1.125,
        -1,
        -0.125,
        0,
        0.875,
        1,
        1.875,
        2,
        3,
        4,
        5,
        6,
        7};
};

#endif // LSLIDAR_CH128X1_PROTOCOL_HPP