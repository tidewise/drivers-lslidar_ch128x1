#include <boost/test/unit_test.hpp>
#include <lslidar_ch128x1/Dummy.hpp>

using namespace lslidar_ch128x1;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    lslidar_ch128x1::DummyClass dummy;
    dummy.welcome();
}
