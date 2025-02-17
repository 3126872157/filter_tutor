//
// Created by ken on 25-2-17.
//

#ifndef KALMAN_H
#define KALMAN_H

#include <Eigen/Dense>

class Kalman
{
private:
    //状态变量
    Eigen::Vector3d x;

    //状态转移矩阵
    Eigen::Matrix3d A;
    //观测矩阵
    Eigen::Matrix<double, 1, 3> H;

    //估计值与真实值误差的协方差
    Eigen::Matrix3d P;
    //过程噪声协方差
    Eigen::Matrix3d Q;
    //观测噪声协方差
    Eigen::Matrix<double, 1, 1> R;

    double T;

public:
    //初始化
    Kalman(double _time);
    //设置Q，R矩阵
    void Q_set(double qx);
    void R_set(double rx);

    Eigen::Vector3d predict();
    Eigen::Vector3d update(const Eigen::VectorXd& z_measure);
};


#endif //KALMAN_H
