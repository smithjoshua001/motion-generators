#pragma once
#include <string>
#include <Eigen/Dense>
#include <CosimaUtilities/Timing.hpp>
#include "motion-generators/JointTrajectory.hpp"

template <typename T> class JointDynamicAttractor : public JointTrajectory<T> {
public:
    JointDynamicAttractor() : JointTrajectory<T>() {
        prev_time = 0;
        sim_period = -1;
        finished = true;
        limit = 0.001;
    }
    JointDynamicAttractor(size_t dof) : JointTrajectory<T>(dof) {
        gainMatrix.resize(dof * 2, dof * 2);
        gainMatrix.setZero();
        state.resize(dof * 2);
        state.setZero();
        state_dot.resize(dof * 2);
        state_dot.setZero();
        des_state.resize(dof * 2);
        des_state.setZero();
        accelerationOffset.resize(dof);
        accelerationOffset.setZero();
        prev_time = 0;
        sim_period = -1;
        finished = true;
    }

    void setInitialState(Eigen::Matrix<T, Eigen::Dynamic, 1> initialState) {
        state = initialState;
    }
    void setAccelerationOffset(const Eigen::Matrix<T, Eigen::Dynamic, 1> &accoff) {
        this->accelerationOffset = accoff;
    }
    Eigen::Matrix<T, Eigen::Dynamic, 1> &getInitialState() {
        return state;
    }
    void setDesiredState(Eigen::Matrix<T, Eigen::Dynamic, 1> desiredState) {
        des_state = desiredState;
    }
    Eigen::Matrix<T, Eigen::Dynamic, 1> &getDesiredState() {
        return des_state;
    }

    void setGain(T gain) {
        gainMatrix.block(0, this->dof, this->dof, this->dof).setIdentity();

        gainMatrix.block(this->dof, 0, this->dof, this->dof).setIdentity();
        gainMatrix.block(this->dof, 0, this->dof, this->dof) *= -gain * gain;

        gainMatrix.block(this->dof, this->dof, this->dof, this->dof).setIdentity();
        gainMatrix.block(this->dof, this->dof, this->dof, this->dof) *= -2 * gain;
        this->gain = gain;
    }

    void setSimulationPeriod(T sim_period) {
        this->sim_period = sim_period;
    }

    void setFinished(bool finished) {
        this->finished = finished;
    }

    void setDof(size_t dof) override {
        JointTrajectory<T>::setDof(dof);
        gainMatrix.resize(dof * 2, dof * 2);
        gainMatrix.setZero();
        state.resize(dof * 2);
        state.setZero();
        state_dot.resize(dof * 2);
        state_dot.setZero();
        des_state.resize(dof * 2);
        des_state.setZero();
        accelerationOffset.resize(dof);
        accelerationOffset.setZero();
    }

    void update(double time = 0) override {
        assert(this->runnable);
        diff_time = time - prev_time;
        prev_time = time;
        state_dot = gainMatrix * (state - des_state);
        state_dot.head(this->dof) += des_state.tail(this->dof);
        state_dot.tail(this->dof) += accelerationOffset;

        state = state + state_dot * diff_time;
        
        this->position = state.head(this->dof);
        this->velocity = state.tail(this->dof);
        this->acceleration = state_dot.tail(this->dof);
        // std::cout<<limit<<std::endl;
        finished = ((state - des_state).cwiseAbs().array() < limit).all();
    }
    T getPeriodLength() {
        return sim_period;
    };

    void loadFromJSON(std::string filename) override {
        JointTrajectory<T>::loadFromJSON(filename);
        // RAPIDJSON_NAMESPACE::Document doc;
        // this->dof = T(doc["dof"].GetInt());
        // setDof(this->dof);
        // T gain = T(doc["gain"].GetDouble());
        // setGain(gain);

        // this->runnable = true;
        this->dof = this->json["dof"].number_value();
        setDof(this->dof);
        T gain = T(this->json["gain"].number_value());
        setGain(gain);

        this->runnable = true;
    }

    bool finished;

    void setLimit(T limit) {
        this->limit = limit;
    }

private:
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> gainMatrix;
    Eigen::Matrix<T, Eigen::Dynamic, 1> state, state_dot;
    Eigen::Matrix<T, Eigen::Dynamic, 1> des_state;
    Eigen::Matrix<T, Eigen::Dynamic, 1> accelerationOffset;
    double prev_time, diff_time;
    T sim_period;
    T limit;
    T gain;
};
