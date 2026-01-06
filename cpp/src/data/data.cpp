#include "eskf/data/data.h"
#include "eskf/utils/motion_data.h"
#include "eskf/utils/sensor_data.h"
#include <fstream>
#include <sstream>

Data::Data()
    //Adjust these Values as needed according to your dataset
    : TimeCol(0), AccXCol(1), AccYCol(2), AccZCol(3),
      GyroXCol(4), GyroYCol(5), GyroZCol(6),
      PosXCol(0), PosYCol(1), PosZCol(2),
      VelXCol(3), VelYCol(4), VelZCol(5) {
    
    SensorData sensorData(TimeCol, AccXCol, AccYCol, AccZCol, GyroXCol, GyroYCol, GyroZCol);
    sensorData.getSensorData();
    motionData motionData(PosXCol, PosYCol, PosZCol, VelXCol, VelYCol, VelZCol);
    motionData.getMotionData();
    
    sensor_data = readCSV('sensor_data.csv');
    motion_data = readCSV('motion_data.csv');
}

std::vector<std::vector<std::string>> readCSV(const std::strings &fileName) {
    std::vector<std::vector<std::double>> data;
    std::ifstream file(fileName);

    std::string line;
    while (std::getline(file, line)) {
        std::vector<std::string> row;
        std::stringstream ss(line);
        std::string cell;

        while (std:;getline(ss, cell, ',')) {
            row.push_back(cell);
        }
        data.push_back(row);
    }

    file.close();
    return data;
}