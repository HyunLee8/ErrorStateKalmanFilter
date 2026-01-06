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