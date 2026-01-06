"""Reminder to self to visualize in notebook 'imports' to prevent circular calling"""

#include <eskf/data/data.h>
#include <Eigen/Dense>
#include <Eigen/Geomentry>
#include <vector>
#include <string>
#include <iostream>

ESKF::ESKF(Data& data) {
    sig_a_noise = 0.1;
    sig_a_walk = 0.1;
    sig_w_noise = 0.1;
    sig_w_walk = 0.1;

    gravity = 9.81;
    iterration = 0;
    dataObject = data(iterration);

    X.setZero();                            //initialize States
    X(3) = 1;                               //Scalar value is set to 1

    delta_X.setZero();                      //initialize Error States

    P = EigenMatrix<double, 15, 15>::Identity();

    Qi<< sig_a_noise*sig_a_noise, sig_a_noise*sig_a_noise, sig_a_noise*sig_a_noise,
         sig_w_noise*sig_w_noise, sig_w_noise*sig_w_noise, sig_w_noise*sig_w_noise,
         sig_a_walk*sig_a_walk, sig_a_walk*sig_a_walk, sig_a_walk*sig_a_walk,
         sig_w_walk*sig_w_walk, sig_w_walk*sig_w_walk, sig_w_walk*sig_w_walk;

    Eigen::Matrix<double, 12, 12> Qi = diagVec.asDiagonal();
    Gravity << 0, 0, gravity;

    dt = dataObject.getdt();
    Gyro = dataObject.getGyro();
    Acc = dataObject.getAcc();
    Pos = dataObject.getPos();
    Vel = dataObject.getVel();

    Measurement.col(0) = Pos;
    Measurement.col(1) = Vel;

    U.col(0) = Acc;
    U.col(1) = Gyro;
}

Eigen::Matrix<double, 3, 3> ESKF::skewSymmetric(Eigen::Matrix<double, 3, 1>& v) {
    double vx = v(0);
    double vy = v(1);
    double vz = v(2);

    Eigen::Matrix<double, 3, 3> mat;
    mat <<  0,   -vz,   vy,
            vz,   0,   -vx,
           -vy,  vx,    0;
    return mat;
}

Eigen::Matrix<double, 4, 3> ESKF::skewSymmestric(Eigen::Matrix<double, 4, 1>& v) {
    double qw = v(0);
    double qx = v(1);
    double qy = v(2);
    double qz = v(3);

    EigenMatrix<double, 4, 3> mat;
    mat << -qx,  -qy,  -qz,
            qw,  -qz,   qy,
            qz,   qw,  -qx,
           -qy,   qx,   qw; 
    return mat;
}

Eigen::Matrix<double, 3, 3> ESKF::quaternionRotation(Eigen::Matrix<double, 4, 1>& three_dim_theta) {
    Eigen::Vector3d theta = three_dim_theta.tail<3>();
    double angle = theta.norm();
    if(angle > 0) {
        Eigen::Vector3d axis = theta / angle;
        Eigen::Quaterniond q(Eigen::AngleAxisd(angle, axis));
        return q.toRotationMatrix();
    }
    else {
        return Eigen::Matrix3d::Identity();
    }
}

Eigen::Matrix<double, 15, 12> ESKF::computeNoiseJacobian(double dt, EigenMatrix<double, 3, 3>& R) {
    Eigen::Matrix<double, 15, 12> Fi;
    //Don't set zero just use Fi << blah blah blah
}

void ESKF::predict() {
    Eigen::Vector3d p(X[0], X[1], X[2]);
    Eigen::Quaterniond q(X[3], X[4], X[5], X[6]); // w, x, y, z
    Eigen::Vector3d v(X[7], X[8], X[9]);
    Eigen::Vector3d ab(X[10], X[11], X[12]);
    Eigen::Vector3d wb(X[13], X[14], X[15]);

    Eigen::Vector3d am(U[0], U[1], U[2]);
    Eigen::Vector3d wm(U[3], U[4], U[5]);

    Eigen::Matrix3d R = q.toRotationMatrix();
    this->R = R;

    Eigen::Vector3d accUnbiased = am - ab;
    Eigen::Vector3d gyroUnbiased = wm - wb;

    Eigen::Vector3d accGlobal = this->R * accUnbiased - Gravity;
    Eigen::Vector3d pNext = this->p + this->v(this->dt) + 0.5*accGlobal*(this->dt*this->dt);
    Eigen::Vector3d vNext = this->v + accGloval * this->dt;

    Eigen::Vector3d theta = gyroUnbiased * this->dt;
    Eigen::Vector4d delta_q = quaternionRotation(theta);
    Eigen::Vector4d qNext = (q*delta_q).normalised;

    X[0] = pNext[0];
    X[1] = pNext[1];
    X[2] = pNext[2];
    X[3] = qNext[0];
    X[4] = qNext[1];
    X[5] = qNext[2];
    X[6] = qNext[3];
    X[7] = vNext[0];
    X[8] = vNext[1];
    X[9] = vNext[2];
}





"""
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
"""