//
// Created by ken on 25-2-17.
//

#include "kalman_filter/kalman.h"

Kalman::Kalman(const double _time) : T(_time)
{
    A << 1, T, 0.5 * T * T,
        0, 1, T,
        0, 0, 1;

    H << 1, 0, 0;

    P.setIdentity();

    x.setZero();
}

void Kalman::Q_set(const double qx)
{
    Eigen::Vector3d G;
    //高阶余项作为误差来源
    G << T * T * T / 6.0, T * T / 2.0, T;
    //协方差公式，qx为放大因数，可调
    Q = G * qx * G.transpose();
}

void Kalman::R_set(const double rx)
{
    R << rx;
}

//预测
Eigen::Vector3d Kalman::predict()
{
    //根据先验的状态估计方程预测状态
    x = A * x;
    //预测误差
    P = A * P * A.transpose() + Q;
    return x;
}

//更新，注意z_measure的维度
Eigen::Vector3d Kalman::update(const Eigen::VectorXd& z_measure)
{
    //代码整洁考虑，记下Ht
    Eigen::MatrixXd Ht = H.transpose();
    //更新卡尔曼增益
    Eigen::MatrixXd K = P * Ht * (H * P * Ht + R).inverse();
    //更新卡尔曼对状态的估计
    x = x + K * (z_measure - H * x);
    //更新加入观测值后的预测误差协方差
    P = (Eigen::Matrix3d::Identity() - K * H) * P;

    return x;
}
