#include <fstream>
#include <gtest/gtest.h>
#include <iostream>
#include <lslidar_ch128x1/Protocol.hpp>

using namespace lslidar_ch128x1;
using namespace std;

unsigned char* openAndReadFile(std::string const& file_name);

struct ProtocolTest : public ::testing::Test {
    Protocol protocol = Protocol();
};

TEST_F(ProtocolTest, it_parses_a_msop_packet_single_echo_mode)
{
    unsigned char* data = openAndReadFile(string("/MSOP.bin"));

    protocol.handleSingleEcho(data);
    auto point_cloud = protocol.getPointCloud();

    ASSERT_NEAR(point_cloud.points[0].x(), -0.331206, 1e-3);
    ASSERT_NEAR(point_cloud.points[0].y(), 9.11941, 1e-3);
    ASSERT_NEAR(point_cloud.points[0].z(), -0.515942, 1e-3);
    ASSERT_EQ(point_cloud.points.size(), 88);
}

TEST_F(ProtocolTest, it_returns_a_point_cloud_if_the_start_marker_was_received)
{
    unsigned char* data_with_marker = openAndReadFile(string("/MSOP_with_marker.bin"));
    unsigned char* data = openAndReadFile(string("/MSOP.bin"));

    protocol.handleSingleEcho(data);
    auto point_cloud = protocol.handleSingleEcho(data_with_marker);

    ASSERT_EQ(point_cloud->points.size(), 88);
}

TEST_F(ProtocolTest, it_returns_nullopt_if_the_start_marker_wasnt_received)
{
    unsigned char* data = openAndReadFile(string("/MSOP.bin"));

    auto point_cloud = protocol.handleSingleEcho(data);

    ASSERT_FALSE(point_cloud.has_value());
}

TEST_F(ProtocolTest, it_parses_a_difop_message)
{
    unsigned char* data = openAndReadFile(string("/DIFOP.bin"));

    protocol.handleDIFOP(data);
    auto configuration = protocol.getConfiguration();

    ASSERT_EQ("192.168.1.200", configuration.lidar_ip);
    ASSERT_EQ("192.168.1.102", configuration.computer_ip);
    ASSERT_EQ(599, configuration.motor_speed);
    ASSERT_EQ(0, configuration.lidar_stationary);
    ASSERT_EQ(0, configuration.clock_source);
    ASSERT_EQ(0, configuration.standby_mode);
    ASSERT_EQ(0, configuration.phase_lock_enabled);
    ASSERT_EQ(0, configuration.phase_lock_angle.getDeg());
    ASSERT_EQ("50:3e:7c:20:a1:43", configuration.mac_address);
    ASSERT_EQ(2368, configuration.data_port);
    ASSERT_EQ(2369, configuration.device_port);
    ASSERT_EQ(1, configuration.flow_packet_interval);
}

unsigned char* openAndReadFile(std::string const& file_name)
{
    ifstream file(DATA_SRC_DIR + file_name, ios::binary | ios::ate);
    streamsize size = file.tellg();
    file.seekg(0, ios::beg);
    unsigned char* data = new unsigned char[size];
    file.read(reinterpret_cast<char*>(data), size);
    file.close();
    return data;
}