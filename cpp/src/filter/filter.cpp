"""Reminder to self to visualize in notebook 'imports' to prevent circular calling"""

#include <eskf/filter/filter.h>
#include <eskf/data/data.h>
#include <eskf/utils/sensor_data.h>
#include <eskf/utils/motion_data.h>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <iostream>

ESKF::ESKF(Data& data) {
    sig_a_noise = 0.1;
    sig_a_walk = 0.1;
    sig_w_noise = 0.1;
    sig_w_walk = 0.1;

    dataObject = data;
    gravity = 9.81;
    iterration = 0;

    X.setZero();
    X(3) = 1;

    delta_X.setZero();
    P = EigenMatrix<double, 15, 15>::Identity();

    Qi<< sig_a_noise*sig_a_noise, sig_a_noise*sig_a_noise, sig_a_noise*sig_a_noise,
         sig_w_noise*sig_w_noise, sig_w_noise*sig_w_noise, sig_w_noise*sig_w_noise,
         sig_a_walk*sig_a_walk, sig_a_walk*sig_a_walk, sig_a_walk*sig_a_walk,
         sig_w_walk*sig_w_walk, sig_w_walk*sig_w_walk, sig_w_walk*sig_w_walk;

    Eigen::Matrix<double, 12, 12> Qi = diagVec.asDiagonal();
    Gravity << 0, 0, gravity;
}







class ESKF {
    private:
        //Adjust the parameters
        double sig_a_noise = 0.1;
        double sig_a_walk = 0.1;
        double sig_w_noise = 0.1;
        double sig_w_walk = 0.1;

        auto dataObject = null;
        double gravity = 9.81;
        int iterration = 0;

        Eigen::Matrix3d X;
        Eigen::Matrix3d delta_X;
        Eigen::Matrix3d P;
        Eigen::Matrix3d Qi;
        Eigen::Matrix3d gravity;
    
        double Gyro, Acc, dt;
        double Pos, Vel;
    
        Eigen::Matri3d Measurement;
        Eigen::Matrix3d U;
        Eigen::Matrix3d R;
    
    public:
        ESKF(Data& data) {
            dataObject = data;
            X = Eigen::Matrix3d
        }

        void skewSymmetric(Eigen::Matrix& v);
    
        void quaternionSkewSymmetric(Eigen::& q);
    
        void qRot(Eigen::Vector& theta);
    
        void computeNoiseJacobian(int& dt, Eigen::Matrix& R);
    
        void computeErrorStateJacobian(int& dt, Eigen::Matrix& a, EigenMatrix& w, EigenMatrix& R);
    
        void predict();
    
        void update(Eigen::Matrix& RMeasurement)
    }

