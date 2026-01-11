"""
Author: Isaac Lee
Date: Jan 10, 2026
Description:
    Here is the sensor data object. From data.cpp
    I passed value 'i' in order to iterrate through
    the data frame. In getSensorData Acc and Gyro
    are returned as a Eigen::Matrix in a 3x1 format.
    You could also use Eigen::Vector3d but for the 
    sake of consistency almost all the 3d vectors 
    were written in Eigen::Matrix from the start 
    because I thought I had to reshape it later.
    Time step will be returned as a double.
"""

#include "eskf/utils/sensor_data.h"

SensorData::SensorData(const std::vector<std::vector<double>>& data)
    : sensorData(data){}

void SensorData::getSensorData(int i) {
    dt = sensorData[i+1][DT_COL] - sensorData[i][DT_COL];

    Acc << sensorData[i][ACC_X_COL],
           sensorData[i][ACC_Y_COL],
           sensorData[i][ACC_Z_COL];

    Gyro << sensorData[i][GYRO_X_COL],
            sensorData[i][GYRO_Y_COL],
            sensorData[i][GYRO_Z_COL]; 
}

double SensorData::getdt() {
    return dt;
}

Eigen::Matrix<double, 3, 1> SensorData::getAcc() {
    return Acc;
}

Eigen::Matrix<double, 3, 1> SensorData::getGyro() {
    return Gryo;
}