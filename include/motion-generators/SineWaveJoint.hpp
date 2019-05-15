#pragma once
#include <string>
#include <Eigen/Dense>
#include <CosimaUtilities/Timing.hpp>
#include "motion-generators/JointTrajectory.hpp"

template <typename T> class SineWaveJoint : public JointTrajectory<T> {
public:
    SineWaveJoint() : JointTrajectory<T>() {}
    SineWaveJoint(size_t dof) : JointTrajectory<T>(dof) {
        // this->dof = dof;
        freq.resize(this->dof);
        freq.setZero();
        scale.resize(this->dof);
        scale.setZero();
        offset.resize(this->dof);
        offset.setZero();
        // freq = (Eigen::Matrix<T, Eigen::Dynamic, 1>::Random(this->DOF));
        // freq += T(1.0);
        // scale = (Eigen::Matrix<T, Eigen::Dynamic, 1>::Random(this->DOF));
        // scale += T(1.0);
        // scale /= T(2.0);
        this->simulation_period = 0;
    }

    template<typename Derived> void setFrequencies(const Eigen::ArrayBase<Derived> &frequencies) {
        freq = frequencies;
        this->simulation_period = 2 * M_PI / freq.maxCoeff();
    }

    template<typename Derived> void setScales(const Eigen::ArrayBase<Derived> &scales) {
        scale = scales;
    }

    T getPeriodLength() {
        return this->simulation_period;
    }

    void setDof(size_t dof) override {
        JointTrajectory<T>::setDof(dof);
        // freq = (Eigen::Matrix<T, Eigen::Dynamic, 1>::Random(this->DOF));
        // freq += T(1.0);
        // scale = (Eigen::Matrix<T, Eigen::Dynamic, 1>::Random(this->DOF));
        // scale += T(1.0);
        // scale /= T(2.0);
        freq.resize(this->dof);
        freq.setOnes();
        scale.resize(this->dof);
        scale.setOnes();
        offset.resize(this->dof);
        offset.setZero();
        // std::cout << "FREQUENCY: " << freq.transpose() << std::endl << "Size: " << freq.size() << std::endl;
        this->simulation_period = 2 * M_PI / freq.eval().maxCoeff();
        this->runnable = true;
    }

    void update(double time = 0) override {
        assert(this->runnable);
        this->position = scale * (freq * time).sin() + offset;
        this->velocity = scale * freq * (freq * time).cos();
        this->acceleration = -scale * freq * freq * (freq * time).sin();
    }

    void loadFromJSON(std::string filename) override {
        // std::cout << "SINEWAVE JSON!!" << std::endl;
        JointTrajectory<T>::loadFromJSON(filename);
        // // rapidjson::Document doc;
        // std::cout << "SINEWAVE JSON!" << std::endl;
        // // this->dof = size_t(this->doc["dof"].GetInt());
        // // setDof(this->dof);

        // // for (size_t i = 0; i < this->dof; i++) {
        // //     freq(i) = T(this->doc["frequency"][i].GetDouble());
        // //     scale(i) = T(this->doc["scale"][i].GetDouble());
        // // }
        // // this->simulation_period = 2 * M_PI / freq.maxCoeff();

        // // this->runnable = true;

        // std::cout << this->json.dump() << std::endl;
        this->dof = this->json["dof"].number_value();
        setDof(this->dof);
        freq.resize(this->dof);
        scale.resize(this->dof);
	offset.resize(this->dof);
        for (size_t i = 0; i < this->dof; i++) {
            freq(i) = T(this->json["frequency"][i].number_value());
            scale(i) = T(this->json["scale"][i].number_value());
            offset(i) = T(this->json["offset"][i].number_value());
        }
        this->simulation_period = 2 * M_PI / freq.maxCoeff();

        this->runnable = true;
    }

private:
    Eigen::Array<T, Eigen::Dynamic, 1> freq;
    Eigen::Array<T, Eigen::Dynamic, 1> scale;
    Eigen::Array<T, Eigen::Dynamic, 1> offset;

    T simulation_period;
};
