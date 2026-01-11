"""
Author: Isaac Lee
Date: Jan 10, 2026
Description:
    Here the implementation is the same structure as Sensor_data. 
    Only difference is getting the Position and Velocity from
    the gps. Also no returning a double value.
"""

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