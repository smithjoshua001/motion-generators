#pragma once
#include <string>
#include <Eigen/Dense>
#include <CosimaUtilities/Timing.hpp>

template <typename T> class JointDynamicAttractor : public JointTrajectory<T> {
public:
    JointDynamicAttractor() : JointTrajectory<T>() {
        prev_time = 0;
        sim_period = -1;
        finished = true;
    }
    JointDynamicAttractor(size_t dof) : JointTrajectory<T>(dof) {
        this->DOF = dof;
        gainMatrix.resize(dof * 2, dof * 2);
        gainMatrix.setZero();
        state.resize(dof * 2);
        state.setZero();
        state_dot.resize(dof * 2);
        state_dot.setZero();
        des_state.resize(dof * 2);
        des_state.setZero();
        prev_time = 0;
        sim_period = -1;
        finished = true;
    }

    void setInitialState(Eigen::Matrix<T, Eigen::Dynamic, 1> initialState) {
        state = initialState;
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
        gainMatrix.block(0, DOF, DOF, DOF).setIdentity();

        gainMatrix.block(DOF, 0, DOF, DOF).setIdentity();
        gainMatrix.block(DOF, 0, DOF, DOF) *= -gain * gain;

        gainMatrix.block(DOF, DOF, DOF, DOF).setIdentity();
        gainMatrix.block(DOF, DOF, DOF, DOF) *= -2 * gain;
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
    }

    void update(double time = 0) override {
        assert(this->runnable);
        diff_time = time - prev_time;
        prev_time = time;
        state_dot = gainMatrix * (state - des_state);
        state = state + state_dot * diff_time;
        this->position = state.head(DOF);
        this->velocity = state.tail(DOF);
        this->acceleration = state_dot.tail(DOF);
        finished = (state - des_state).cwiseAbs().norm() < 0.001;
    }
    T getPeriodLength() {
        return sim_period;
    };

    void loadFromJSON(std::string filename) override {
        std::ifstream ifs(filename);
        rapidjson::IStreamWrapper isw(ifs);
        rapidjson::Document doc;
        doc.ParseStream(isw);
        if (!doc.IsObject()) {
            std::cerr << "Config File: " << filename << " does not exist for JointDynamicAttractor" << std::endl;
            exit(-2);
        }
        this->dof = T(doc["dof"].GetInt());
        setDof(this->dof);
        T gain = T(doc["gain"].GetDouble());
        setGain(gain);

        this->runnable = true;
    }

    bool finished;

private:
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> gainMatrix;
    Eigen::Matrix<T, Eigen::Dynamic, 1> state, state_dot;
    Eigen::Matrix<T, Eigen::Dynamic, 1> des_state;
    size_t DOF;
    double prev_time, diff_time;
    T sim_period;
};
