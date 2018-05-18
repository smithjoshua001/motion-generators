#pragma once
#include <Eigen/Dense>

template <typename T> class CartesianTrajectory<T> {
private:
    Eigen::Matrix<T, 7, 1> pose;
    Eigen::Matrix<T, 6, 1> velocity;
    Eigen::Matrix<T, 6, 1> acceleration;
public:
    virtual Eigen::Matrix<T, 7, 1> &getPose(double time = 0);
    virtual Eigen::Matrix<T, 6, 1> &getVelocity(double time = 0);
    virtual Eigen::Matrix<T, 6, 1> &getAcceleration(double time = 0);
}
