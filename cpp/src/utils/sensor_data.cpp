#include "eskf/utils/sensor_data.h"

SensorData::SensorData(int time, int accx, int accy, int accz, int gyrox, int gyroy, int gyroz)
    : Time(time), Accx(accx), AccY(accy), AccZ(accz), GyroX(gyrox), GyroY(gyroy), GyroZ(gyroz) {} //cpp initalizes like this. You can also do assignments but apparently this is normal.

void SensorData::getSensorData(int i, double& Time, Eigen::Vector3d& Acc, Eigen::Vector3d& Gyro) {
    Time << sensorData[i][Time];

    Acc << sensorData[i][AccX],
           sensorData[i][AccY],
           sensorData[i][AccZ];

    Gyro << sensorData[i][GyroX],
            sensorData[i][GyroY],
            sensorData[i][GyroZ]; 
}

void MotionData::loadData(const std::vector<std::vector<std::string>>& data) {
    sensorData = data;
}