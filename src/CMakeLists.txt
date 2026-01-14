cmake_minimum_required(VERSION 3.16)
project(ErrorStateKalmanFilter)

set(CMAKE_CXX_STANDARD 17)

add_executable(ErrorStateKalmanFilter
    src/main.cpp
    src/utils/motion_data.cpp
    src/include/eskf/utils/motion_data.h
    src/utils/sensor_data.cpp
    src/include/eskf/utils/sensor_data.h
    src/utils/RunFilter.cpp
    src/include/eskf/utils/RunFilter.h
    src/filter/Filter.cpp
    src/include/eskf/filter/Filter.h
    src/data/data.cpp
    src/include/eskf/data/data.h
)