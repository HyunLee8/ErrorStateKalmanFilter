#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include <Eigen/Dense>
#include <vector>
#include <utility>

class SensorData {
private:
    std::vector<std::vector<double>> sensorData;
    Eigen::Matrix<double, 3, 1> Acc;
    Eigen::Matrix<double, 3, 1> Gyro;
    double dt;

    static constexpr int DT_COL = 0;
    static constexpr int ACC_X_COL = 1;
    static constexpr int ACC_Y_COL = 2;
    static constexpr int ACC_Z_COL = 3;
    static constexpr int GYRO_X_COL = 4;
    static constexpr int GYRO_Y_COL = 5;
    static constexpr int GYRO_Z_COL = 6;

public:
    SensorData(const std::vector<std::vector<std::double>>& data);

    void getSensorData(int i);

    double getdt();

    Eigen::Matrix<double, 3, 1> getAcc();

    Eigen::Matrix<double, 3, 1> getGyro();
};

#endif