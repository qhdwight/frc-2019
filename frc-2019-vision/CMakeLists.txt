cmake_minimum_required(VERSION 3.3)
project(frc-2019-vision)
set(CMAKE_CXX_STANDARD 14)

include_directories(raspbian-frc/include)

add_executable(program src/program.cpp raspbian-frc/include)
