#ifndef MOTION_DATA_H
#define MOTION_DATA_H

#include <Eigen/Dense>
#include <vector>
#include <utility>

class MotionData {
private:
    std::vector<std::vector<std::double>> motionData;
    Eigen::Matrix<double, 3, 1> Pos;
    Eigen::Matrix<double, 3, 1> Vel;

    static constexpr int POS_X_COL = 0;
    static constexpr int POS_Y_COL = 1;
    static constexpr int POS_Z_COL = 2;
    static constexpr int VEL_X_COL = 3;
    static constexpr int VEL_Y_COL = 4;
    static constexpr int VEL_Z_COL = 5;

public:
    MotionData(const std::vector<std::vector<double>>& data);

    void getMotionData(int i);

    Eigen::Matrix<double, 3, 1> getPos;

    Eigen::Matrix<double, 3, 1> getVel;
}