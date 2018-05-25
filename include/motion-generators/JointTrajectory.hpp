#pragma once
#include <Eigen/Dense>
#include <exception>
#include <tuple>
#include <iostream>

// #define rapidjson my::rapid::json
// #define rapidjson_BEGIN namespace my { namespace rapid { namespace json {
// #define rapidjson_END } } }

#include <fstream>
#include <memory>
#include <string>
#include <streambuf>
#include <json11.hpp>

#include "motion-generators/Trajectory.hpp"

template <typename T, int DOF_C = -1> class JointTrajectory : Trajectory<T, DOF_C, DOF_C, DOF_C> {
protected:
    size_t dof;
public:

    JointTrajectory() : Trajectory<T, DOF_C, DOF_C, DOF_C>() {}

    JointTrajectory(size_t dof) : Trajectory<T, DOF_C, DOF_C, DOF_C>() {
        this->dof = dof;
        position.resize(this->dof);
        position.setZero();
        velocity.resize(this->dof);
        velocity.setZero();
        acceleration.resize(this->dof);
        acceleration.setZero();
        runnable = true;
    }
    JointTrajectory(size_t dof, Eigen::Matrix<T, DOF_C, 1> position) {
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

    TrajectoryStorage<T> simulate(T control_frequency) {
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

    virtual void setDof(size_t dof) {

        this->dof = dof;
        velocity.resize(this->dof);
        position.resize(this->dof);
        acceleration.resize(this->dof);
        position.setZero();
        acceleration.setZero();
        velocity.setZero();
    }
    virtual void loadFromJSON(std::string filename) {
        Trajectory<T, pos_dim, vel_dim, acc_dim>::loadFromJSON(filename);
        this->setDof(json["dof"].int_value());
    }
};
