#include "Protocol.hpp"
#include <lslidar_ch128x1/Protocol.hpp>
#include <vector>

using namespace lslidar_ch128x1;
using namespace base::samples;
using namespace std;

double toRad(double degree);

Protocol::Protocol()
{
    computeDefaultSinesAndCosines();
}

void Protocol::getOffsetAngle(std::vector<int> const& prism_angles)
{
    double prismAngle[4];
    float PrismOffsetVAngle = 0.0f;

    // judge if there is any offset angle. 0: no vertical offset angle; other value: there
    // is vertical offset angle
    if (abs(prism_angles[0]) != 0) {
        PrismOffsetVAngle = prism_angles[0] / 100.0;

        for (int i = 0; i < 128; i++) {

            /*
                Distinguish the spots in the left or right； for the left, the  vertical
               offset angle should be added; for the right, there is no change.
            */
            // left
            if (i / 4 % 2 == 0) {
                m_sin_theta_1[i] = sin(EMISSION_ANGLES_DEGREE[i / 4] + PrismOffsetVAngle);
                m_cos_theta_1[i] = cos(EMISSION_ANGLES_DEGREE[i / 4] + PrismOffsetVAngle);
            }
            else {
                m_sin_theta_1[i] = sin(EMISSION_ANGLES_DEGREE[i / 4]);
                m_cos_theta_1[i] = cos(EMISSION_ANGLES_DEGREE[i / 4]);
            }
        }
    }
    else {
        // follow the preset value if there is no offset angle
        for (int i = 0; i < 128; i++) {
            m_sin_theta_1[i] = sin(EMISSION_ANGLES_DEGREE[i / 4]);
            m_cos_theta_1[i] = cos(EMISSION_ANGLES_DEGREE[i / 4]);
        }
    }

    // judge if the prism needs calibration offset, all values are 0: default prism angle
    // which is 0; -0.17; -0.34; -0.51
    if (abs(prism_angles[1]) == 0 && abs(prism_angles[2]) == 0 &&
        abs(prism_angles[3]) == 0 && abs(prism_angles[4]) == 0) {

        for (int i = 0; i < 4; i++) {
            prismAngle[i] = i * -0.17;
        }

        for (int i = 0; i < 128; i++) {
            m_sin_theta_2[i] = sin(prismAngle[i % 4]);
            m_cos_theta_2[i] = cos(prismAngle[i % 4]);
        }
    }
    else {
        for (int i = 0; i < 4; i++) {
            prismAngle[i] = prism_angles[i + 1] / 100.0;
        }

        // add the prism angle in the device packet to the calculation, figure out the
        // sine value and consine value first.
        for (int i = 0; i < 128; i++) {
            m_sin_theta_2[i] = sin(prismAngle[i % 4]);
            m_cos_theta_2[i] = cos(prismAngle[i % 4]);
        }
    }
}

void Protocol::handleDIFOP(unsigned char* data)
{
    if (data[44] == 0x00) {
        m_time_service_mode = TimeServiceMode::GPS;
    }
    else if (data[44] == 0x01) {
        m_time_service_mode = TimeServiceMode::GPTP;
    }

    m_utc_time.year = data[52] + 2000;
    m_utc_time.month = data[53];
    m_utc_time.day = data[54];
    m_utc_time.hour = data[55];
    m_utc_time.minute = data[56];
    m_utc_time.second = data[57];

    std::vector<int> prism_angles;
    for (int i = 0; i < 5; i++) {
        short int prism_angle = data[240 + i * 2] * 256 + data[241 + i * 2];
        prism_angles.emplace_back(std::move(prism_angle));
    }
    getOffsetAngle(prism_angles);
}

base::Point Protocol::getPoint(uint8_t line_number,
    double horizontal_angle_degree,
    double distance)
{
    double vertical_angle_sin =
        computeVerticalAngleSin(line_number, horizontal_angle_degree);
    double vertical_angle_cos = sqrt(1 - vertical_angle_sin * vertical_angle_sin);
    base::Point point;
    point.x() = distance * vertical_angle_cos * cos(toRad(horizontal_angle_degree));
    point.y() = distance * vertical_angle_cos * sin(toRad(horizontal_angle_degree));
    point.z() = distance * vertical_angle_sin;
    return point;
}

base::samples::Pointcloud Protocol::handleSingleEcho(unsigned char* data)
{
    uint64_t timestamp_microseconds =
        (static_cast<uint64_t>(data[1200] << 24) +
            static_cast<uint64_t>(data[1201] << 16) +
            static_cast<uint64_t>(data[1202] << 8) + data[1203]);
    int miliseconds = timestamp_microseconds / 1000;
    int microseconds = timestamp_microseconds % 1000;
    base::Time time = base::Time::fromTimeValues(m_utc_time.year,
        m_utc_time.month,
        m_utc_time.day,
        data[1197],
        data[1198],
        data[1199],
        miliseconds,
        microseconds);
    Pointcloud point_cloud;
    point_cloud.time = time;
    for (int i = 0; i < 1197; i = i + 7) {
        if (data[i] < 128) {
            double distance =
                (data[i + 3] * 65536 + data[i + 4] * 256 + data[i + 5]) / 256.0 / 100;
            uint8_t line_number = data[i];
            double horizontal_angle_degree = (data[i + 1] * 256 + data[i + 2]) / 100.f;
            uint8_t intensity = data[i + 6];

            auto point = getPoint(line_number, horizontal_angle_degree, distance);
            point_cloud.points.push_back(point);
            point_cloud.colors.push_back(base::Vector4d(intensity, 0, 0, 0));
        }
    }
    return point_cloud;
}

double toRad(double degree)
{
    return (degree * M_PI / 180);
}

double Protocol::computeVerticalAngleSin(uint8_t line_number,
    double horizontal_angle_degree)
{
    double r = m_cos_theta_2[line_number] * m_cos_theta_1[line_number] *
                   cos(toRad(horizontal_angle_degree / 2.0)) -
               m_sin_theta_2[line_number] * m_sin_theta_1[line_number];
    double sin_vertical_angle =
        m_sin_theta_1[line_number] + 2 * r * m_sin_theta_2[line_number];
    return sin_vertical_angle;
}

void Protocol::computeDefaultSinesAndCosines()
{
    for (int i = 0; i < 128; i++) {
        m_sin_theta_1[i] = sin(toRad(EMISSION_ANGLES_DEGREE[i / 4]));
        m_sin_theta_2[i] = sin(toRad((i % 4) * (-0.17)));
        m_cos_theta_1[i] = cos(toRad(EMISSION_ANGLES_DEGREE[i / 4]));
        m_cos_theta_2[i] = cos(toRad((i % 4) * (-0.17)));
    }
}
