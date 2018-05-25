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
    bool runnable;
public:

    JointTrajectory() {
        runnable = false;
    }

    JointTrajectory(size_t dof) {
        this->dof = dof;
        position.resize(this->dof);
        position.setZero();
        velocity.resize(this->dof);
        velocity.setZero();
        acceleration.resize(this->dof);
        acceleration.setZero();
        runnable = true;
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
        runnable = true;
    }
    virtual ~JointTrajectory() {};
    virtual void update(double time = 0) = 0;
    Eigen::Matrix<T, Eigen::Dynamic, 1> const &getPosition() {return position;}
    Eigen::Matrix<T, Eigen::Dynamic, 1> const &getVelocity() {return velocity;}
    Eigen::Matrix<T, Eigen::Dynamic, 1> const &getAcceleration() {return acceleration;}
    virtual T getPeriodLength() = 0;

    Trajectory<T> simulate(T control_frequency) {
        assert(runnable);
        //TODO better solution
        assert(this->getPeriodLength() > 0 && "Invalid Trajectory to simulate");
        int simulated_points = std::ceil(this->getPeriodLength() * control_frequency);
        // Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> pos, vel, acc;
        trajectory.pos.resize(this->dof, simulated_points);
        trajectory.vel.resize(this->dof, simulated_points);
        trajectory.acc.resize(this->dof, simulated_points);
        for (int i = 0; i < simulated_points; i++) {
            // std::cout << ((T)i) / control_frequency << "\n";
            this->update(((T)i) / control_frequency);
            trajectory.pos.col(i) = this->getPosition();
            trajectory.vel.col(i) = this->getVelocity();
            trajectory.acc.col(i) = this->getAcceleration();
        }
        return trajectory;
    }
    size_t getDof() {
        return dof;
    }

    bool &getRunnable() {
        return runnable;
    }

    virtual void setDof(size_t dof) {
        this->dof = dof;
        position.resize(this->dof);
        velocity.resize(this->dof);
        acceleration.resize(this->dof);
        position.setZero();
        velocity.setZero();
        acceleration.setZero();
    }
    virtual void loadFromJSON(std::string filename) {}
};
