#ifndef MOTION_DATA_H
#define MOTION_DATA_H

#include <Eigen/Dense>
#include <vector>
#include <utility>

class MotionData {
private:
    std::vector<std::vector<std::double>> motionData;
    int PosX, PosY, PosZ;
    int velX, VelY, VelZ;

public:
    MotionData(int posx, int posy, int posz, int velx, int vely, int velz);

    void getMotionData(int i, Eigen::Vector& Pos, Eigen::Vector& Vel);

    void loadData(const std::vector<std::vector<double>>& data);
}