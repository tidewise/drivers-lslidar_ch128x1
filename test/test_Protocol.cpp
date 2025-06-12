#include <fstream>
#include <gtest/gtest.h>
#include <lslidar_ch128x1/Protocol.hpp>

using namespace lslidar_ch128x1;
using namespace std;

struct ProtocolTest : public ::testing::Test {};

TEST_F(ProtocolTest, it_parses_a_msop_packet_single_echo_mode)
{
    ifstream file(DATA_SRC_DIR + string("/MSOP.bin"), ios::binary | ios::ate);
    streamsize size = file.tellg();
    file.seekg(0, ios::beg);
    unsigned char* data = new unsigned char[size];
    file.read(reinterpret_cast<char*>(data), size);
    file.close();

    auto protocol = Protocol();
    protocol.handleSingleEcho(data);
    auto point_cloud = protocol.getPointCloud();
    ASSERT_NEAR(point_cloud.points[0].x(), -0.331206, 1e-3);
    ASSERT_NEAR(point_cloud.points[0].y(), 9.11941, 1e-3);
    ASSERT_NEAR(point_cloud.points[0].z(), -0.515942, 1e-3);
    ASSERT_EQ(point_cloud.points.size(), 88);
}

TEST_F(ProtocolTest, it_returns_a_point_cloud_if_the_start_marker_was_received)
{
}

TEST_F(ProtocolTest, it_returns_nullopt_if_the_start_marker_wasnt_received)
{
}