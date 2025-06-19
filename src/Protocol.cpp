#include "Protocol.hpp"
#include <iomanip>
#include <lslidar_ch128x1/Protocol.hpp>
#include <vector>

using namespace lslidar_ch128x1;
using namespace base::samples;
using namespace std;

Protocol::Protocol()
{
    for (unsigned int i = 0; i < 32; i++) {
        m_emission_angles.push_back(base::Angle::fromDeg(EMISSION_ANGLES_DEGREE[i]));
    }
    computeDefaultSinesAndCosines();
    m_full_frame = false;
}

void Protocol::getOffsetAngle(std::vector<int> const& prism_angles)
{
    /**
     * Judge if there is any offset angle. 0: no vertical offset angle; other value: there
     * is vertical offset angle
     */
    if (abs(prism_angles[0]) != 0) {
        float prism_offset_vertical_angle_degree = prism_angles[0] / 100.0;
        base::Angle prism_offset_vertical_angle =
            base::Angle::fromDeg(prism_offset_vertical_angle_degree);
        for (int i = 0; i < 128; i++) {
            /**
             * Distinguish the spots in the left or right； for the left, the  vertical
             * offset angle should be added; for the right, there is no change.
             */
            // left
            if (i / 4 % 2 == 0) {
                m_sin_theta_1[i] = sin(
                    (m_emission_angles[i / 4] + prism_offset_vertical_angle).getRad());
                m_cos_theta_1[i] = cos(
                    (m_emission_angles[i / 4] + prism_offset_vertical_angle).getRad());
            }
            else {
                m_sin_theta_1[i] = sin((m_emission_angles[i / 4]).getRad());
                m_cos_theta_1[i] = cos((m_emission_angles[i / 4]).getRad());
            }
        }
    }
    else {
        // Follow the preset value if there is no offset angle
        for (int i = 0; i < 128; i++) {
            m_sin_theta_1[i] = sin((m_emission_angles[i / 4]).getRad());
            m_cos_theta_1[i] = cos((m_emission_angles[i / 4]).getRad());
        }
    }
    /**
     * Judge if the prism needs calibration offset, all values are 0: default prism angle
     * which is 0; -0.17; -0.34; -0.51
     */
    base::Angle prism_angle[4];
    if (abs(prism_angles[1]) == 0 && abs(prism_angles[2]) == 0 &&
        abs(prism_angles[3]) == 0 && abs(prism_angles[4]) == 0) {
        for (int i = 0; i < 4; i++) {
            prism_angle[i] = base::Angle::fromDeg(i * -0.17);
        }
        for (int i = 0; i < 128; i++) {
            m_sin_theta_2[i] = sin((prism_angle[i % 4]).getRad());
            m_cos_theta_2[i] = cos((prism_angle[i % 4]).getRad());
        }
    }
    else {
        for (int i = 0; i < 4; i++) {
            prism_angle[i] = base::Angle::fromDeg(prism_angles[i + 1] / 100.0);
        }
        /**
         * Add the prism angle in the device packet to the calculation, figure out the
         * sine value and consine value first.
         */
        for (int i = 0; i < 128; i++) {
            m_sin_theta_2[i] = sin((prism_angle[i % 4]).getRad());
            m_cos_theta_2[i] = cos((prism_angle[i % 4]).getRad());
        }
    }
}

std::string parseIP(unsigned char* data)
{
    std::string ip;
    for (size_t i = 0; i <= 3; i++) {
        ip += to_string(data[i]);
        if (i != 3) {
            ip += ".";
        }
    }
    return ip;
}

uint16_t fromBigEndian(unsigned char* data)
{
    return static_cast<uint16_t>(data[0]) << 8 | static_cast<uint16_t>(data[1]) << 0;
}

base::Angle parsePhaseLockAngle(unsigned char* data)
{
    uint16_t value = fromBigEndian(data);
    return base::Angle::fromDeg(value / 100);
}

TimeUTC parseUTC(unsigned char* data)
{
    TimeUTC time_utc;
    time_utc.year = data[52] + 2000;
    time_utc.month = data[53];
    time_utc.day = data[54];
    time_utc.hour = data[55];
    time_utc.minute = data[56];
    time_utc.second = data[57];
    return time_utc;
}

std::string parseMACAddr(unsigned char* data)
{
    std::ostringstream mac_stream;
    for (int i = 0; i < 6; ++i) {
        if (i != 0) {
            mac_stream << ":";
        }
        mac_stream << std::hex << std::setw(2) << std::setfill('0')
                   << static_cast<int>(data[i]);
    }
    return mac_stream.str();
}

gps_base::Position parseGPSPosition(unsigned char* data)
{
    // TODO: Since we still don't have a hardware integration between the sensor and the
    // $GPRMC sender, we couldn't test this parse yet. The documentation is not very clear
    // on this point
    return gps_base::Position();
}

void Protocol::handleDIFOP(unsigned char* data)
{
    m_configuration.motor_speed = fromBigEndian(data + 8);
    m_configuration.lidar_ip = parseIP(data + 10);
    m_configuration.computer_ip = parseIP(data + 14);
    m_configuration.mac_address = parseMACAddr(data + 18);
    m_configuration.data_port = fromBigEndian(data + 24);
    m_configuration.device_port = fromBigEndian(data + 26);
    m_configuration.lidar_stationary = data[41];
    m_configuration.flow_packet_interval = static_cast<FlowPacketInterval>(data[43]);
    m_configuration.clock_source = static_cast<TimeServiceMode>(data[44]);
    m_configuration.standby_mode = data[45];
    m_configuration.phase_lock_enabled = data[46];
    m_configuration.phase_lock_angle = parsePhaseLockAngle(data + 47);
    m_configuration.time_utc = parseUTC(data);

    std::vector<int> prism_angles;
    for (int i = 0; i < 5; i++) {
        short int prism_angle = data[240 + i * 2] * 256 + data[241 + i * 2];
        prism_angles.emplace_back(std::move(prism_angle));
    }
    getOffsetAngle(prism_angles);
}

base::samples::Pointcloud Protocol::getPointCloud()
{
    return m_point_cloud;
}

base::Point Protocol::getPoint(uint8_t line_number,
    base::Angle const& horizontal_angle,
    double distance)
{
    double vertical_angle_sin = computeVerticalAngleSin(line_number, horizontal_angle);
    double vertical_angle_cos = sqrt(1 - vertical_angle_sin * vertical_angle_sin);
    base::Point point;
    point.x() = distance * vertical_angle_cos * cos(horizontal_angle.getRad());
    point.y() = distance * vertical_angle_cos * sin(horizontal_angle.getRad());
    point.z() = distance * vertical_angle_sin;
    return point;
}

base::Vector4d colorByReflectivity(uint8_t intensity)
{
    if (intensity < 30) {
        return base::Vector4d(0,
            static_cast<int>(intensity * 255 / 30) & 0xff,
            0xff,
            0xff);
    }
    else if (intensity < 90) {
        return base::Vector4d(0,
            0xff,
            static_cast<int>((90 - intensity) * 255 / 60) & 0xff,
            0xff);
    }
    else if (intensity < 150) {
        return base::Vector4d(static_cast<int>((intensity - 90) * 255 / 60) & 0xff,
            0xff,
            0,
            0xff);
    }
    else {
        return base::Vector4d(0xff,
            static_cast<int>((255 - intensity) * 255 / (256 - 150)) & 0xff,
            0,
            0xff);
    }
}

bool isStartMarker(unsigned char* data)
{
    return data[0] == 0xff && data[1] == 0xaa && data[2] == 0xbb && data[3] == 0xcc &&
           data[4] == 0xdd;
}

double computeDistance(unsigned char* data)
{
    return (data[3] * 65536 + data[4] * 256 + data[5]) / 256.0 / 100;
}

std::optional<base::samples::Pointcloud> Protocol::handleSingleEcho(unsigned char* data)
{
    if (m_full_frame == true) {
        m_point_cloud.points.clear();
        m_point_cloud.colors.clear();
        m_full_frame = false;
    }
    uint64_t timestamp_microseconds =
        (static_cast<uint64_t>(data[1200] << 24) +
            static_cast<uint64_t>(data[1201] << 16) +
            static_cast<uint64_t>(data[1202] << 8) + data[1203]);
    int miliseconds = timestamp_microseconds / 1000;
    int microseconds = timestamp_microseconds % 1000;
    base::Time time = base::Time::fromTimeValues(m_configuration.time_utc.year,
        m_configuration.time_utc.month,
        m_configuration.time_utc.day,
        data[1197],
        data[1198],
        data[1199],
        miliseconds,
        microseconds);
    for (int i = 0; i < 1197; i = i + 7) {
        if (isStartMarker(data + i)) {
            m_full_frame = true;
            // TODO: Use the syncronized time here. We are using the time::now() at this
            // moment cause we need a hardware integration between the lidar and a $GPRMC
            // sender
            m_point_cloud.time = base::Time::now();
            return m_point_cloud;
        }
        // data[i] is the vertical line, so it can't be higher than 127
        if (data[i] < 128) {
            double distance = computeDistance(data + i);
            if (distance != 0) {
                uint8_t line_number = data[i];
                base::Angle horizontal_angle =
                    base::Angle::fromDeg((data[i + 1] * 256 + data[i + 2]) / 100.f);

                uint8_t intensity = data[i + 6];

                auto point = getPoint(line_number, horizontal_angle, distance);
                m_point_cloud.points.push_back(point);
                m_point_cloud.colors.push_back(colorByReflectivity(intensity));
            }
        }
    }
    return {};
}

bool isDIFOPMarker(unsigned char* data)
{
    return data[0] == 0xa5 && data[1] == 0xff && data[2] == 0x00 && data[3] == 0x5a;
}

std::optional<base::samples::Pointcloud> Protocol::handleData(unsigned char* data)
{
    if (isDIFOPMarker(data)) {
        handleDIFOP(data);
        return {};
    }
    else {
        return handleSingleEcho(data);
    }
}

double Protocol::computeVerticalAngleSin(uint8_t line_number,
    base::Angle const& horizontal_angle)
{
    double r = m_cos_theta_2[line_number] * m_cos_theta_1[line_number] *
                   cos(horizontal_angle.getRad() / 2.0) -
               m_sin_theta_2[line_number] * m_sin_theta_1[line_number];
    double sin_vertical_angle =
        m_sin_theta_1[line_number] + 2 * r * m_sin_theta_2[line_number];
    return sin_vertical_angle;
}

void Protocol::computeDefaultSinesAndCosines()
{
    for (int i = 0; i < 128; i++) {
        m_sin_theta_1[i] = sin((m_emission_angles[i / 4]).getRad());
        m_sin_theta_2[i] = sin((base::Angle::fromDeg((i % 4) * (-0.17))).getRad());
        m_cos_theta_1[i] = cos((m_emission_angles[i / 4]).getRad());
        m_cos_theta_2[i] = cos((base::Angle::fromDeg((i % 4) * (-0.17))).getRad());
    }
}

Configuration Protocol::getConfiguration()
{
    return m_configuration;
}
