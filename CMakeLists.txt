# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 3.9)
project(lslidar_ch128x1
        VERSION 0.1
        DESCRIPTION "")
find_package(Rock)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

rock_init()
rock_standard_layout()
