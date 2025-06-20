#include "TestHelpers.hpp"
#include <gtest/gtest.h>
#include <iodrivers_base/FixtureGTest.hpp>
#include <lslidar_ch128x1/Driver.hpp>
#include <lslidar_ch128x1/Protocol.hpp>

using namespace lslidar_ch128x1;
using namespace std;

struct DriverTest : public ::testing::Test, iodrivers_base::Fixture<Driver> {
    void openDriver()
    {
        driver.openURI("test://");
    }

    void pushDataToDriver(std::vector<uint8_t> const& msg)
    {
        iodrivers_base::Fixture<Driver>::pushDataToDriver(msg);
    }

    void pushDataToDriver(pair<unsigned char*, size_t> msg)
    {
        std::vector<uint8_t> packet(msg.first, msg.first + msg.second);
        iodrivers_base::Fixture<Driver>::pushDataToDriver(packet);
    }
};

TEST_F(DriverTest, it_parses_msop_packets)
{
    auto data = lslidar_ch128x1::openAndReadFile(string("/MSOP.bin"));
    auto data_with_marker =
        lslidar_ch128x1::openAndReadFile(string("/MSOP_with_marker.bin"));
    pushDataToDriver(data);
    driver.read();
    pushDataToDriver(data_with_marker);
    auto point_cloud = driver.read();
    ASSERT_EQ(point_cloud->points.size(), 88);
}

TEST_F(DriverTest, it_parses_difop_packets)
{
    auto data = lslidar_ch128x1::openAndReadFile(string("/DIFOP.bin"));
    pushDataToDriver(data);
    driver.read();
    auto configuration = driver.protocol.getConfiguration();
    ASSERT_EQ(configuration.computer_ip, "192.168.1.102");
}
