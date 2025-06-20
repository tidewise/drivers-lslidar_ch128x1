#include "TestHelpers.hpp"

using namespace lslidar_ch128x1;
using namespace std;

pair<unsigned char*, size_t> lslidar_ch128x1::openAndReadFile(
    std::string const& file_name)
{
    ifstream file(DATA_SRC_DIR + file_name, ios::binary | ios::ate);
    streamsize size = file.tellg();
    file.seekg(0, ios::beg);
    unsigned char* data = new unsigned char[size];
    file.read(reinterpret_cast<char*>(data), size);
    file.close();
    return make_pair(data, size);
}
