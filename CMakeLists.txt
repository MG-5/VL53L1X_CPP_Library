cmake_minimum_required(VERSION 3.21)
project(VL53L1X_CPP_Library)

set(CMAKE_CXX_STANDARD 14)

include_directories(src)

add_executable(VL53L1X_CPP_Library
        src/SparkFun_VL53L1X.cxx
        src/vl53l1x_class.cxx)