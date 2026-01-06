#include "eskf/utils/sensor_data.h"

SensorData::SensorData(const std::vector<std::vector<double>>& data)
    : sensorData(data){}

void SensorData::getSensorData(int i) {
    Time = sensorData[i][TIME_COL];

    Acc << sensorData[i][ACC_X_COL],
           sensorData[i][ACC_Y_COL],
           sensorData[i][ACC_Z_COL];

    Gyro << sensorData[i][GYRO_X_COL],
            sensorData[i][GYRO_Y_COL],
            sensorData[i][GYRO_Z_COL]; 
}

SensorData::double getTime() {
    return Time;
}

SensorData::Eigen::Matrix getAcc() {
    return Acc;
}

SensorData::Eigen::Matrix getGyro() {
    return Gryo;
}