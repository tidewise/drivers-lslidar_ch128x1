rock_library(lslidar_ch128x1
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(lslidar_ch128x1_bin Main.cpp
    DEPS lslidar_ch128x1)

