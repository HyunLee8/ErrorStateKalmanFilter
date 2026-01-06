#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include <Eigen/Dense>
#include <vector>
#include <utility>

class SensorData {
private:
    std::vector<std::vector<std::string>> sensorData;
    int Time;
    int AccX, AccY, AccZ;
    int GyroX, GyroY, GyroZ;

public:
    SensorData(int time, int accx, int accy, int accz, int gyrox, int gyroy, int gyroz);
    
    void getSensorVectors(int i, double& Time, Eigen::Vector3d& Acc, Eigen::Vector3d& Gyro);

    void loadData(const std::vector<std::vector<std::string>>& data);
}

#endif