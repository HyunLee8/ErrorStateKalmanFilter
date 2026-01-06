#include "eskf/utils/motion_data/h"

MotionData::MotionData(const std::vector<std::vector<double>>& data)
    : motionData(data) {}

void MotionData::getMotionData(int i) {
    Pos << motionData[i][PosX],
           motionData[i][PosY],
           motionData[i][PosZ];
    
    Vel << motionData[i][VelX],
           motionData[i][VelY],
           motionData[i][VelZ];
}

Eigen::Matrix<double, 3, 1> SensorData::getPos() {
    return Pos;
}

Eigen::Matrix<double, 3, 1> SensorData::getVel() {
    return Vel;
}