#pragma once
#include <Eigen/Dense>
#include <motion-generators/JointTrajectory.hpp>
#include <motion-generators/ModifiedFourierTrajectory.hpp>
#include <random>
#include <cmath>
#include <iostream>
#include <Eigen/StdVector>

#include <rapidjson/document.h>
#include <rapidjson/ostreamwrapper.h>
#include <rapidjson/istreamwrapper.h>
#include <rapidjson/prettywriter.h>

template <typename T> class MultiModifiedFourierTrajectory : public JointTrajectory<T> {
private:
    std::vector<ModifiedFourierTrajectory<T> > MFTs;
    std::vector<ModifiedFourierTrajectory<T> > test_MFTs;
    T period;
    size_t trajectory_number;
    size_t test_trajectory_number;

public:
    MultiModifiedFourierTrajectory() : JointTrajectory<T>() {}

    MultiModifiedFourierTrajectory(size_t dof) : ModifiedFourierTrajectory<T>(dof) {}

    ~MultiModifiedFourierTrajectory() override {}

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    void loadFromJSON(std::string filename) override {
        this->runnable = true;
        JointTrajectory<T>::loadFromJSON(filename);
        this->json["trajectory_number"].int_value();
        this->json["test_trajectory_number"].int_value();
    }

    void update(double time = 0) override {
        assert(this->runnable);
    }

    void display() {}
    T getPeriodLength() override {
        return period;
    }
};
