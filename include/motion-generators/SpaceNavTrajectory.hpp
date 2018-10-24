#pragma once
#include <Eigen/Dense>
#include <string>
#include "motion-generators/CartesianTrajectory.hpp"

template <typename T> class SpaceNavTrajectory : public HumanInputTrajectory<T>{
SpaceNavTrajectory() : HumanInputTrajectory<T>() {
    currentPose.setZero();
    desiredPose.setZero();
    maxDistance = 0.1;
}
~SpaceNavTrajectory() {}
void loadFromJSON(std::string filename) override {
    CartesianTrajectory<T>::loadFromJSON(filename);
}

void update(double dt) override {
    //check and shrink distance (position only first)
    errorPosition = (desiredPose.head<3>() - currentPose.head<3>());
    if (errorPosition.norm() > maxDistance) {
        desiredPose.head<3>() = currentPose.head<3>() + (errorPosition * (maxDistance / errorPosition.norm()));
    }
    //TODO add orientation limits
}

template <typedef Derived> void setInitialPose(const Eigen::MatrixBase<Derived> &pose) {
    currentPose = pose;
}

template <typedef Derived> void setDesiredPose(const Eigen::MatrixBase<Derived> &pose) {
    desiredPose = pose;
}

protected:
    Eigen::Matrix<T, 3, 1> errorPosition;
    Eigen::Matrix<T, 7, 1> currentPose, desiredPose;
    T maxDistance;
};
