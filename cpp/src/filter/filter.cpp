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

    delta_X.setZero();                      //initialize Error States (this is LOWER CASE DELTA. Not change in Time))

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
    RMeasurment = nullptr;

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

void ESKF::quaternionRotation(Eigen::Matrix<double, 4, 1>& three_dim_theta) {
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

Eigen::Matrix<double, 15, 15> ESKF::computeErrorStateJacobian(double& dt, Eigen::Vector3d& a, Eigen::Vector3d& w, Eigen::Matrix3d& R) {
    Eigen::Matrix<double, 15, 15> Fx = Eigen::Matrix<double, 15, 15>::Identity();
    Fx.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity() * dt;
    Fx.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() - skewSymmetric(w) * dt;
    Fx.block<3, 3>(3, 12) = -Eigen::Matrix3d::Identity() * dt;
    Fx.block<3, 3>(6, 3) = -R * skewSymmetric(a) * dt;
    Fx.block<3, 3>(6, 9) = -R * dt;

    return Fx;
}

Eigen::Matrix<double, 15, 15> ESKF::computeNoiseJacobian(double dt, const EigenMatrix3d& R) {
    Eigen::Matrix<double, 15, 12> Fi;
    Fi.setZero();
    Fi.block<3, 3>(6, 0) = -R * dt;
    Fi.block<3, 3>(3, 3) = -Eigen::Matrix3d::Identity() * dt;
    Fi.block<3, 3>(9, 6) = Eigen::Matrix3d::Identity() * dt;
    Fi.block<3, 3>(12, 9) = Eigen::Matrix3d::Identity() * dt;

    return Fi;
}

void ESKF::predict() {
"""
    Summary of how the prediction step works --
        Phase 1 in prediction step ~~ variable prediction:

            All values of the State variables start at either 0 or any default value.
            As the motion of the sensor data changes through time -> gryo and acc, we can
            estmate true acceleration by subtracting measured acc and measured gyro from acc bias 
            and gyro bias. Getting true acc helps attain global acceleration with respect to the world.
            Now perform a matrix multiplication of a rotated orientation matrix minus the gravity 
            since it is wrt Earth. With true acc you can now use the fundamental kinematic
            formulas to get a predicted velocity and position. Now in order to predict orientation, take 
            gyro, multiply by time step to get delta_theta and transform those euler values into quaternions
            to get change in quaternions. Now we have delta_q and multiply it by the current quaternions.

        Phase 2 in prediciton step ~~ jacobian calculations and correlations:

            Changes in one variable may have an impact on another variable. However some variables do NOT have
            impact on others. The jacobian matrix comes in handy when determining how dependent certain varibles
            are to eachother and thus not all changes in variables may change others of the same magnitude.
            By getting noise jacobian and error jacobian we can compute Error covariance. Imagine two variables
            that are correlated to each other but we want to predict how much; by calculating the covariance,
            it will calculate an average point of correlation and produce a guassian distribution and on a graph,
            that would look like a cloud that gets more dense in the middle. Covariance would be represent how far
            out the cloud goes. This is then passed in the update step; continued in update step ~

    VARIBLES

        X: [p, q, v ab, wb] state vector
        P: error covariance matrix
        U: [ax ay az wx wy wz]  IMU body input vector
        dt: time step
"""
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

    Fx = computeErrorStateJacobian(dt, accUnbiased, gyroUnbiased, this->R);
    Fi = computeNoiseJacobian(dt, this->R);

    P = Fx * P * Fx.transpose() + Fi * Qi * Fi.transpose();

    delta_q.setZero();
    iterration++;
}

void update() {
"""
    Summary of how Update step works --
        Phase 1 in Update Step ~~ Sensor fusion:

            Whether it is GPS, SLAM, or altimeter data, in order to incorporate an update step it needs 
            some sort of global stabalizer. If you were to use a drone you might use images and process
            it through ORB at a lower frequency as a global stabalizer. In this ESKF we will use GPS data
            to work with. Remeber that guassian cloud I was reffering from earlier? Well how we use that 
            in the update step is by getting that 'prediction cloud' (from earlier) comparing it against 
            sensor data H that has it's own cloud, combine the two clouds by multiplying each point in the
            guassian and producing a new gaussian. That is Kalman gain. You would now update the predicted 
            covarience for recycling.

        Phase 2 in Update Step ~~ calculate delta/error variables:

            Before talking about adding the error state variables, everytime the prediction or update step
            is complete we have to reset the error state to 0. This is because the small changes are never
            influenced by the previous calculations. Remeber that this is a local stabalizer. You can
            get the error values by performing a matrix multiplication with the kalman gain. After that 
            setting the variables is straight forward with the exception of quaternions. You have to convert
            euler error values into quaternions but luckily pyquaternion exists and saves a shit ton of time.
            Now just multiply the error quaternions to the current quaternions and then just normlise it so
            all values add to 1. Now just run it with each row in a data set and boom you now have a fully
            function Error State Kalman Filter
"""
    Eigen::Vector3d p(X[0], X[1], X[2]);
    Eigen::Quaterniond q(X[3], X[4], X[5], X[6]); // w, x, y, z
    Eigen::Vector3d v(X[7], X[8], X[9]);
    Eigen::Vector3d ab(X[10], X[11], X[12]);
    Eigen::Vector3d wb(X[13], X[14], X[15]);

    Eigen::MatrixXd H;

    Eigen::VectorXd flatMeasurement = Eigen::Map<const Eigen::VectorXd> (
        Measurement.data(), 
        Measurement.rows() * Measurement.cols()
    );

    int measuredSize = 6 //IMPORTANT: if you ONLY have position you MUST change this to 3

    if(RMeasurement == null) {
        R_measurement = Eigen::MatrixXd::Identity(measuredSize, measuredSize) * 0.1;
    }

    if(measuredSize == 3) {
        H = Eigen::MatrixXd::Zero(3, 15);
        H.block<3, 3>(0, 0) = Eigne::Matrix3d::Identity();
        y = Eigen::MatrixXd(3, 1);
        Eigen::Map<Eigen::Matrix<double, 3, 1>
        y = Measurement - p; // IMPORNTANT: I don't think i have to reshape here but I will see during compile

    } else if(measuredSize == 6) {
        H = Eigen::MatrixXd::Zero(6, 15);
        H.block<3, 3>(0, 0) = Eigne::Matrix3d::Identity();
        H.block<3, 3>(3, 6) = Eigne::Matrix3d::Identity();
        Eigen::Matrix<double, 6, 1> predicted; // IMPORNTANT: same issue as line 235
        predicted << p,
                     v;
        y = Measurement - predicted; //FIX THIS LATER.
    } else {
        std::cout << "Measured Size is not applicable" << '\n';
        break;
    }

    Eigen::Matrix<double, measuredSize, measuredSize> S = H * P * H.transpose() + RMeasurement;
    Eigen::Matrix<double, 15, measuredSize> K = P * H.transpose() * S.ldlt().solve(Eigen::Matrix3d::Identity());

    delta_X = (K * y);

    Eigen::Matrix<double, 15, 15> I_KH = Eigen::Matrix<double, 15, 15>::Identity() - K * H;
    P = I_KH * P * I_KH.transpose() + K * RMeasurement * K.transpose();

    Eigen::Vector3d delta_p(delta_X[0], delta_X[1], delta_X[2]);
    Eigen::Vector3d delta_theta(delta_X[3], delta_X[4], delta_X[5]);
    Eigen::Vector3d delta_v(delta_X[6], delta_X[7], delta_X[8]);
    Eigen::Vector3d delta_ab(delta_X[9], delta_X[10], delta_X[11]);
    Eigen::Vector3d delta_wb(delta_X[12], delta_X[13], delta_X[14]);

    Eigen::Quaterniond nominalQ(X[0], X[1], X[2], X[3]);
    double angle = delta_theta.norm();

    if angle < 1e-8 {
        delta_q = Eigen::Quaterniond(1, 0.5*delta_theta[0], 0.5*delta_theta[1], 0.5*delta_theta[2]);
    } else {
        Eigen::Vector3d axis = delta_theta / angle;
        delta_q = Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
    }

    Eigen::Quaterniond updatedQ = (delta_q * nominalQ).normalized();

    X[0] += delta_p[0];
    X[1] += delta_p[1];
    X[2] += delta_p[2];
    X[3] = updatedQ.w();
    X[4] = updatedQ.x();
    X[5] = updatedQ.y();
    X[6] = updatedQ.z();
    X[7] += delta_v[0];
    X[8] += delta_v[1];
    X[9] += delta_v[2];
    X[10] += delta_ab[0];
    X[11] += delta_ab[1];
    X[12] += delta_ab[2];
    X[13] += delta_wb[0];
    X[14] += delta_wb[1];
    X[15] += delta_wb[2];
    delta_X.setZero();

"""FIRST DRAFT COMPLETE LETS GO"""
}