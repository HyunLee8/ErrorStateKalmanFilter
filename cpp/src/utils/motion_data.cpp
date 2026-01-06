#include "eskf/utils/motion_data/h"

MotionData::MotionData(int posx, int posy, int posz, int velx, int vely, int velz)
    : PosX(posx), PosY(posy), PosZ(posz), VelX(velx), VelY(vely), VelZ(velz) {}

void MotionData::getMotionData(int i, Eigen::Vector3d& Pos, Eigen::Vector3d& Vel) {
    Pos << motionData[i][PosX],
           motionData[i][PosY],
           motionData[i][PosZ];
    
    Vel << motionData[i][VelX],
           motionData[i][VelY],
           motionData[i][VelZ];
}

void MotionData:loadData(const std::vector<std::vector<double>& data) {
    motionData = data;
}