"""
Author: Isaac Lee
Date: Jan 10, 2026
Description:
    For preprocessing data, I have a local
    readCSV method implemented manually. C++
    does not have a built in read csv like
    python pandas. To prevent jumbled code,
    I implemented two seperate classes for 
    parsing sensor data and motion data. 
    From there you can call the objects' 
    values and return. Data takes a requred
    arg 'i' in order to iterrate through
    the data frame.
"""

#include "eskf/data/data.h"
#include "eskf/utils/motion_data.h"
#include "eskf/utils/sensor_data.h"
#include <fstream>
#include <sstream>

Data::Data(int i) {
    sensor_data_csv = readCSV('sensor_data.csv');
    motion_data_csv = readCSV('motion_data.csv');

    SensorData sensorData(sensor_data_csv);
    sensorData.getSensorData(i);
    motionData motionData(sensor_data_csv);
    motionData.getMotionData(i);
}

double Data::getdt() {
    return sensorData.getdt();
}

Eigen::Matrix<double, 3, 1> Data::getAcc() {
    return sensorData.getAcc();
}

Eigen::Matrix<double, 3, 1> Data::getGyro() {
    return sensorData.getGyro();
}

Eigen::Matrix<double, 3, 1> Data::getPos() {
    return motionData.getPos();
}

Eigen::Matrix<double, 3m 1> Data::getVel() {
    return motionData.getVel();
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