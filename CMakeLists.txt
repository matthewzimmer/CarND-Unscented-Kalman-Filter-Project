project(UnscentedKF)

cmake_minimum_required(VERSION 3.5)

add_definitions(-std=c++0x)

file(COPY ${CMAKE_SOURCE_DIR}/data/sample-laser-radar-measurement-data-1.txt DESTINATION ${CMAKE_BINARY_DIR})
file(COPY ${CMAKE_SOURCE_DIR}/data/sample-laser-radar-measurement-data-2.txt DESTINATION ${CMAKE_BINARY_DIR})

set(sources
        src/ukf.cpp
        src/main.cpp
        src/tools.cpp
        src/LaserMeasurement.cpp
        src/LaserMeasurement.h
        src/RadarMeasurement.cpp
        src/RadarMeasurement.h)

add_executable(UnscentedKF ${sources})
