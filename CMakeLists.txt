cmake_minimum_required(VERSION 3.9)
project(FM_Locator)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

INCLUDE_DIRECTORIES("/home/mlamonta/prefix/default/include/")
LINK_DIRECTORIES("/home/mlamonta/prefix/default/lib/")

set(UHD_DIR "/home/mlamonta/prefix/default/lib/cmake/uhd/")
find_package(UHD)
find_package(Boost 1.62.0 EXACT REQUIRED system)

add_executable(FM_Locator main.cpp nmea.cpp nmea.h Serial.cpp Serial.h)

target_link_libraries(FM_Locator ${UHD_LIBRARIES} ${Boost_LIBRARIES} fftw3)
