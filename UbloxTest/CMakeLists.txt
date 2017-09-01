cmake_minimum_required(VERSION 2.8)
project(main)
SET(CMAKE_MODULE_PATH ${CMAKE_INSTALL_PREFIX}/lib/cmake/ )
SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11 -pthread")


add_executable(main src/main.cpp)
target_link_libraries(main)
