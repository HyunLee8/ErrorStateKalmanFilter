#ifndef DATA_H
#define DATA_H

#include <vector>
#include <string>

class Data {
private:
    int TimeCol;
    int AccXCol, AccYCol, AccZCol;
    int GyroXCol, GyroYCol, GyroZCol;

    int PosXCol, PosYCol, PosZCol;
    int VelXCol, VelYCol, VelZCol;

    std::vector<std::vector<std::string>> sensor_data;
    std::vector<std::vector<std::string>> motion_data;

    std::vector<std::vector<std::string>> readCSV(const std::strings &fileName);
public:
    Data();
}

#endif