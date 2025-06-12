rock_gtest(test_suite suite.cpp
   test_Protocol.cpp
   DEPS lslidar_ch128x1)

target_compile_definitions(test_suite PRIVATE DATA_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")