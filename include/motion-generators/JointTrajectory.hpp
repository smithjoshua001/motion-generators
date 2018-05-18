#pragma once
#include <Eigen/Dense>
#include <exception>
#include <tuple>
#include <iostream>

template <typename T> struct Trajectory {
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> pos, vel, acc;
    Trajectory(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> pos, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> vel, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> acc) {
        this->pos = pos;
        this->vel = vel;
        this->acc = acc;
    }
    Trajectory() {}
};

template <typename T> class JointTrajectory {
protected:
    size_t dof;
    Eigen::Matrix<T, Eigen::Dynamic, 1> position;
    Eigen::Matrix<T, Eigen::Dynamic, 1> velocity;
    Eigen::Matrix<T, Eigen::Dynamic, 1> acceleration;
    Trajectory<T> trajectory;
public:

    JointTrajectory(size_t dof) {
        this->dof = dof;
        position.resize(this->dof);
        position.setZero();
        velocity.resize(this->dof);
        velocity.setZero();
        acceleration.resize(this->dof);
        acceleration.setZero();
    }
    JointTrajectory(size_t dof, Eigen::Matrix<T, Eigen::Dynamic, 1> position) {
        this->dof = dof;
        if (position.size() != dof) {
            throw std::runtime_error("Position dof size does not match the input dof size");
        }
        this->position.resize(this->dof);
        this->position = position;
        velocity.resize(this->dof);
        velocity.setZero();
        acceleration.resize(this->dof);
        acceleration.setZero();
    }
    virtual ~JointTrajectory() {};
    virtual Eigen::Matrix<T, Eigen::Dynamic, 1> const &getPosition(double time = 0) = 0;
    virtual Eigen::Matrix<T, Eigen::Dynamic, 1> const &getVelocity(double time = 0) = 0;
    virtual Eigen::Matrix<T, Eigen::Dynamic, 1> const &getAcceleration(double time = 0) = 0;
    virtual T getPeriodLength() = 0;

    Trajectory<T> simulate(T control_frequency) {
        int simulated_points = std::ceil(this->getPeriodLength() * control_frequency);
        // Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> pos, vel, acc;
        trajectory.pos.resize(this->dof, simulated_points);
        trajectory.vel.resize(this->dof, simulated_points);
        trajectory.acc.resize(this->dof, simulated_points);
        for (int i = 0; i < simulated_points; i++) {
            // std::cout << ((T)i) / control_frequency << "\n";
            trajectory.pos.col(i) = this->getPosition(((T)i) / control_frequency);
            trajectory.vel.col(i) = this->getVelocity(((T)i) / control_frequency);
            trajectory.acc.col(i) = this->getAcceleration(((T)i) / control_frequency);
        }
        return trajectory;
    }
};
