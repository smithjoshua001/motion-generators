#pragma once
#include <Eigen/Dense>
#include <exception>
#include <tuple>
#include <iostream>

#include <fstream>
#include <memory>
#include <string>
#include <streambuf>
#include <json11.hpp>

#include "motion-generators/Trajectory.hpp"

template <typename T, int DOF_C = Eigen::Dynamic> class JointTrajectory : public Trajectory<T, DOF_C, DOF_C, DOF_C> {
protected:
    size_t dof;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    JointTrajectory() : Trajectory<T, DOF_C, DOF_C, DOF_C>() {}

    JointTrajectory(size_t dof) : Trajectory<T, DOF_C, DOF_C, DOF_C>() {
        this->dof = dof;
        this->position.resize(this->dof);
        this->position.setZero();
        this->velocity.resize(this->dof);
        this->velocity.setZero();
        this->acceleration.resize(this->dof);
        this->acceleration.setZero();
    }
    JointTrajectory(size_t dof, Eigen::Matrix<T, DOF_C, 1> position) {
        this->dof = dof;
        if (this->position.size() != dof) {
            throw std::runtime_error("Position dof size does not match the input dof size");
        }
        this->position.resize(this->dof);
        this->position = position;
        this->velocity.resize(this->dof);
        this->velocity.setZero();
        this->acceleration.resize(this->dof);
        this->acceleration.setZero();
        this->runnable = true;
    }
    virtual ~JointTrajectory() {};

    std::shared_ptr<TrajectoryStorage<T, DOF_C, DOF_C, DOF_C> > simulate(T control_frequency) {
        assert(this->runnable);
        //TODO better solution
        assert(this->getPeriodLength() > 0 && "Invalid Trajectory to simulate");
        int simulated_points = std::ceil(this->getPeriodLength() * control_frequency);

        this->trajectory->pos.resize(this->dof, simulated_points);
        this->trajectory->vel.resize(this->dof, simulated_points);
        this->trajectory->acc.resize(this->dof, simulated_points);
        for (int i = 0; i < simulated_points; i++) {
            this->update(((T)i) / control_frequency);
            this->trajectory->pos.col(i) = this->getPosition();
            this->trajectory->vel.col(i) = this->getVelocity();
            this->trajectory->acc.col(i) = this->getAcceleration();
        }
        return this->trajectory;
    }
    size_t getDof() {
        return dof;
    }

    virtual void setDof(size_t dof) {
        this->dof = dof;
        this->position.resize(this->dof);
        this->velocity.resize(this->dof);
        this->acceleration.resize(this->dof);
        this->position.setZero();
        this->velocity.setZero();
        this->acceleration.setZero();
    }
    virtual void loadFromJSON(std::string filename) {
        std::ifstream t(filename);
        std::string str((std::istreambuf_iterator<char>(t)),
                        std::istreambuf_iterator<char>());
        t.close();
        std::string error;
        this->json = json11::Json::parse(str, error);
        if (!error.empty()) {
            std::cerr << "Config File: " << filename << " does not exist for JointDynamicAttractor" << std::endl;
            exit(-2);
        }
    }
};
