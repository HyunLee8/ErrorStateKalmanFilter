#include "eskf/data/data.h"
#include "eskf/utils/motion_data.h"
#include "eskf/utils/sensor_data.h"
#include <fstream>
#include <sstream>

Data::Data() {
    sensor_data_csv = readCSV('sensor_data.csv');
    motion_data_csv = readCSV('motion_data.csv');

    SensorData sensorData(sensor_data_csv);
    sensorData.getSensorData();
    motionData motionData(sensor_data_csv);
    motionData.getMotionData();
}

Data::getTime() {
    return Time;
}

Data::getAcc() {
    return sensorData.
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