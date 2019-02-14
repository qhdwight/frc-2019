cmake_minimum_required(VERSION 3.3)
project(frc-2019-vision)
set(CMAKE_CXX_STANDARD 11)

include_directories(include)

add_executable(program src/streamer.cpp src/calibrator.cpp include)
