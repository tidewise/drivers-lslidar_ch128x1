rock_library(lslidar_ch128x1
    SOURCES Protocol.cpp
            Driver.cpp
    HEADERS Protocol.hpp
            Driver.hpp
            TimeServiceMode.hpp
            TimeUTC.hpp
            Configuration.hpp
            FlowPacketInterval.hpp
    DEPS_PKGCONFIG base-types iodrivers_base gps_base)

rock_executable(lslidar_ch128x1_ctl Main.cpp
    DEPS lslidar_ch128x1)