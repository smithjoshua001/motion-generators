#pragma once
#include <Eigen/Dense>
#include <motion-generators/JointTrajectory.hpp>
#include <motion-generators/ModifiedFourierTrajectory.hpp>
#include <random>
#include <cmath>
#include <iostream>
#include <Eigen/StdVector>
#include <sstream>
#include <json11.hpp>
using namespace json11;

template <typename T> class MultiModifiedFourierTrajectory : public JointTrajectory<T> {
private:
    std::vector<ModifiedFourierTrajectory<T> > MFTs;
    T period;
    size_t trajectory_number;
    size_t executingIndex = 0;
    double modifiedTime = 0, time;
    std::vector<std::string> trajectories;

public:
    MultiModifiedFourierTrajectory() : JointTrajectory<T>() {}
    MultiModifiedFourierTrajectory(std::vector<std::string> &trajectories, size_t dof) : JointTrajectory<T>(dof) {
        this->trajectories = trajectories;
        for (size_t i = 0; i < trajectories.size(); i++) {
            MFTs.push_back(ModifiedFourierTrajectory<T>());
            MFTs.back().loadFromJSON(trajectories[i]);
        }
        trajectory_number = trajectories.size();
        period = 0;
        for (ModifiedFourierTrajectory<T> &mft : MFTs) {
            period += mft.getPeriodLength();
        }
        this->runnable = true;
    }

    ~MultiModifiedFourierTrajectory() override {}

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    void loadFromJSON(std::string filename) override {
        this->runnable = true;
        JointTrajectory<T>::loadFromJSON(filename);
        Json obj = this->json[0];
        if (obj.is_array()) {
            if (obj[0].is_string()) {
                for (size_t i = 0; i < obj.array_items().size(); i++) {
                    MFTs.push_back(ModifiedFourierTrajectory<T>());
                    MFTs.back().loadFromJSON(obj[i].string_value());
                }
                trajectory_number = obj.array_items().size();
            } else if (obj[0].is_object()) {}
        }
        period = 0;
        for (ModifiedFourierTrajectory<T> &mft : MFTs) {
            period += mft.getPeriodLength();
        }
    }

    void update(double time = 0) override {
        assert(this->runnable);
        this->time = time;
        if (time < modifiedTime) {
            modifiedTime = 0;
        }
        if ((time - modifiedTime) >= MFTs[executingIndex].getPeriodLength()) {
            modifiedTime += (time - modifiedTime);//MFTs[executingIndex].getPeriodLength();
            ++executingIndex;
            if (executingIndex >= trajectory_number) {
                executingIndex = 0;
            }
        }
        MFTs[executingIndex].update(time - modifiedTime);
        this->position = MFTs[executingIndex].getPosition();
        this->velocity = MFTs[executingIndex].getVelocity();
        this->acceleration = MFTs[executingIndex].getAcceleration();
    }

    void display() {
        std::cout << "Current trajectory: " << trajectories[executingIndex] << " : " << executingIndex << " : time " << time - modifiedTime << std::endl;
    }
    T getPeriodLength() override {
        return period;
    }
    std::string getInfo() {
        std::stringstream out;
        out << "Current Trajectory : " << executingIndex << " : " << trajectories[executingIndex];
        return out.str();
    }
};
