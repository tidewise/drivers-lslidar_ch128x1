#ifndef LSLIDAR_CH128X1_PROTOCOL_HPP
#define LSLIDAR_CH128X1_PROTOCOL_HPP

#include "lslidar_ch128x1/Configuration.hpp"
#include "lslidar_ch128x1/TimeServiceMode.hpp"
#include "lslidar_ch128x1/TimeUTC.hpp"
#include <base/Float.hpp>
#include <base/samples/Pointcloud.hpp>
#include <optional>

namespace lslidar_ch128x1 {
    class Protocol {
    public:
        /**
         * @brief Parses a MSOP (Main Data Output Protocol) message for single echo mode
         *
         * It contains the point cloud data
         *
         * @param data
         * @return std::optional<base::samples::Pointcloud>
         */
        std::optional<base::samples::Pointcloud> handleSingleEcho(unsigned char* data);
        Protocol();
        /**
         * @brief Parses a DIFOP (Device Information Output Protocol) message
         *
         * It contains the LiDAR configuration
         *
         * @param data
         */
        void handleDIFOP(unsigned char* data);
        base::samples::Pointcloud getPointCloud();
        Configuration getConfiguration();
        /**
         * @brief Identifies and call the right protocol handler
         *
         * @param data
         * @return std::optional<base::samples::Pointcloud>
         */
        std::optional<base::samples::Pointcloud> handleData(unsigned char* data);

    private:
        Configuration m_configuration;
        base::samples::Pointcloud m_point_cloud;
        double m_sin_theta_1[128];
        double m_sin_theta_2[128];
        double m_cos_theta_1[128];
        double m_cos_theta_2[128];
        bool m_full_frame = false;
        std::vector<base::Angle> m_emission_angles;

        /**
         * @brief Calculate the point from the line, the horizontal angle and the distance
         *
         * @param line_number The horizontal line of the point
         * @param horizontal_angle The horizontal angle of the point
         * @param distance The distance of the point to the LiDAR
         * @return base::Point
         */
        base::Point getPoint(uint8_t line_number,
            base::Angle const& horizontal_angle,
            double distance);
        void computeDefaultSinesAndCosines();
        double computeVerticalAngleSin(uint8_t line_number,
            base::Angle const& horizontal_angle);
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