"""
    NOTES 
    LAST PICKUP: 01/06/2025
    CONTINUE: Most helper functions are complete. Continue defining the contrucutor for filter.py and
              fix some of the imports for utils.
"""

#ifndef FILTER_HPP
#define FILTER_HPP

#include "eskf/data/data.h"
#include <vector>
#include <string>
#include <Eigen/Dense>

class ESKF {
private:
    double sig_a_noise;
    double sig_a_walk;
    double sig_w_noise;
    double sig_w_walk;

    int iterration;
    auto dataObject
    double gravity;

    Eigen::Matrix<double, 16, 1> X;
    Eigen::Matrix<double, 15, 1> delta_X;
    Eigen::Matrix<double, 15, 15> P;
    Eigen::Matrix<double, 12, 1> Qi;
    Eigen::Matrix<double, 3, 1> Gravity;

    Eigen::Matrix<double, 3, 1> Gyro
    Eigen::Matrix<double, 3, 1> Acc;
    Eigen::Matrix<double, 3, 1> Pos;
    Eigen::Matrix<double, 3, 1> vel;
    double dt;

    Eigen::Matrix<Eigen::Matrix, 3, 2> Measurement;
    Eigen::Matrix<Eigen::Matrixm 3, 2> U;
    Eigen::Matrix<double, 3, 3> R;

public:
    ESKF(Data& data);

    void skewSymmetric(Eigen::Matrix<double, 3, 3>& v);

    void quaternionSkewSymmetric(Eigen::Matrix<double, 4, 3>& q);

    void qRot(Eigen::Vector& theta);

    void computeNoiseJacobian(int& dt, Eigen::Matrix& R);

    void computeErrorStateJacobian(int& dt, Eigen::Matrix& a, EigenMatrix& w, EigenMatrix& R);

    void predict();

    void update(Eigen::Matrix& RMeasurement)
}