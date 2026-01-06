#ifndef DATA_H
#define DATA_H

#include <vector>
#include <string>

class Data {
private:
    int TimeCol;

    auto sensorData;
    auto motionData;

    double Time;

    double AccXCol, AccYCol, AccZCol;
    double GyroXCol, GyroYCol, GyroZCol;

    double PosXCol, PosYCol, PosZCol;
    double VelXCol, VelYCol, VelZCol;

    std::vector<std::vector<std::double>> sensor_data;
    std::vector<std::vector<std::double>> motion_data;

    std::vector<std::vector<std::double>> readCSV(const std::strings &fileName);
public:
    Data();

    double getTime();
    Eigen::Matrix<double, 3, 1> getAcc();
    Eigen::Matrix<double, 3, 1> getGyro();
    Eigen::Matrix<double, 3, 1> getPos();
    Eigen::Matrix<double, 3, 1> getVel();
    std::vector<std::vector<std::double>> get_sensor_data();
    std::vector<std::vector<std::double>> get_motion_data();
}

#endif