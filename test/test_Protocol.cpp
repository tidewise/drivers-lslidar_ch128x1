#include "TestHelpers.hpp"
#include <fstream>
#include <gtest/gtest.h>
#include <lslidar_ch128x1/Protocol.hpp>

using namespace lslidar_ch128x1;
using namespace std;

struct ProtocolTest : public ::testing::Test {
    Protocol protocol = Protocol();
};

TEST_F(ProtocolTest, it_parses_a_msop_packet_single_echo_mode)
{
    auto data = lslidar_ch128x1::openAndReadFile(string("/MSOP.bin"));

    protocol.handleSingleEcho(data.first);
    auto point_cloud = protocol.getPointCloud();

    ASSERT_NEAR(point_cloud.points[0].x(), -0.331206, 1e-3);
    ASSERT_NEAR(point_cloud.points[0].y(), 9.11941, 1e-3);
    ASSERT_NEAR(point_cloud.points[0].z(), -0.515942, 1e-3);
    ASSERT_EQ(point_cloud.points.size(), 88);
}

TEST_F(ProtocolTest, it_returns_a_point_cloud_if_the_start_marker_was_received)
{
    auto data_with_marker =
        lslidar_ch128x1::openAndReadFile(string("/MSOP_with_marker.bin"));
    auto data = lslidar_ch128x1::openAndReadFile(string("/MSOP.bin"));

    protocol.handleSingleEcho(data.first);
    auto point_cloud = protocol.handleSingleEcho(data_with_marker.first);

    ASSERT_EQ(point_cloud->points.size(), 88);
}

TEST_F(ProtocolTest, it_returns_nullopt_if_the_start_marker_wasnt_received)
{
    auto data = lslidar_ch128x1::openAndReadFile(string("/MSOP.bin"));

    auto point_cloud = protocol.handleSingleEcho(data.first);

    ASSERT_FALSE(point_cloud.has_value());
}

TEST_F(ProtocolTest, it_parses_a_difop_message)
{
    auto data = lslidar_ch128x1::openAndReadFile(string("/DIFOP.bin"));

    protocol.handleDIFOP(data.first);
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

TEST_F(ProtocolTest,
    it_correctly_parses_a_msop_packet_single_echo_mode_after_processing_a_difop)
{
    auto data = lslidar_ch128x1::openAndReadFile(string("/DIFOP.bin"));
    protocol.handleDIFOP(data.first);

    data = lslidar_ch128x1::openAndReadFile(string("/MSOP.bin"));
    protocol.handleSingleEcho(data.first);
    auto point_cloud = protocol.getPointCloud();

    ASSERT_NEAR(point_cloud.points[0].x(), -0.331206, 1e-3);
    ASSERT_NEAR(point_cloud.points[0].y(), 9.11941, 1e-3);
    ASSERT_NEAR(point_cloud.points[0].z(), -0.515942, 1e-3);
    ASSERT_EQ(point_cloud.points.size(), 88);
}
