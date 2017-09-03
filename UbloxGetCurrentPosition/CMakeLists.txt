cmake_minimum_required(VERSION 2.8)
project(main)
SET(CMAKE_MODULE_PATH ${CMAKE_INSTALL_PREFIX}/lib/cmake/ )
SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11 -pthread")

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../libraries/ublox ${CMAKE_CURRENT_BINARY_DIR}/ublox_build)

add_executable(main src/main.cpp)
target_link_libraries(main Ublox)
