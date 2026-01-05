#ifndef FILTER_HPP
#define FILTER_HPP

#include <vector>
#include <string>

class Data {
private:
    int TimeCol;
    int AccXCol;
    int AccYCol;
    int AccZCol;
    int GyroXCol;
    int GyroYCol;
    int GyroZCol;

    int PosXCol;
    int PosYCol;
    int PosZCol;
    int VelXCol;
    int VelYCol;
    int VelZCol;

    std::vector<std::vector<std::string>> sensor_data;
    std::vector<std::vector<std::string>> motion_data;

public:
    int[] get_imu_vectors(int i);
    int[] get_motion_vectors(int i);
}