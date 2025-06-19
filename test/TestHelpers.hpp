#ifndef LSLIDAR_CH128X1_TESTHELPERS_HPP
#define LSLIDAR_CH128X1_TESTHELPERS_HPP

#include <fstream>
#include <string>
#include <vector>

namespace lslidar_ch128x1 {
    std::pair<unsigned char*, size_t> openAndReadFile(std::string const& file_name);
}

#endif
