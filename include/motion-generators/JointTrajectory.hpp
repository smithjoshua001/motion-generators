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

template <typename T> struct Trajectory {
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> pos, vel, acc;
    Trajectory(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> pos, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> vel, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> acc) {
        this->pos = pos;
        this->vel = vel;
        this->acc = acc;
    }
    Trajectory() {}
};

#include <fstream>
#include <memory>
#include <string>
#include <streambuf>
#include <json11.hpp>

#include "motion-generators/Trajectory.hpp"

template <typename T, int DOF_C = -1> class JointTrajectory : Trajectory<T, DOF_C, DOF_C, DOF_C> {
protected:
    size_t dof;
    Eigen::Matrix<T, Eigen::Dynamic, 1> position;
    Eigen::Matrix<T, Eigen::Dynamic, 1> velocity;
    Eigen::Matrix<T, Eigen::Dynamic, 1> acceleration;
    Trajectory<T> trajectory;
    bool runnable;
    // rapidjson::GenericDocument<rapidjson::UTF8<> > doc;
    json11::Json json;
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
        // std::cout << "LOADING JSON" << std::endl;
        // std::ifstream ifs(filename);
        // if (ifs.is_open()) {
        //     std::cout << "OPEN!" << std::endl;
        // } else {
        //     std::cout << "CLOSED!" << std::endl;
        // }
        // std::cout << "IFS " << filename << std::endl;
        // rapidjson::IStreamWrapper isw(ifs);
        // std::cout << "ISW" << std::endl;
        // // doc = rapidjson::Document();
        // // rapidjson::Document doc2;
        // doc = std::make_shared<rapidjson::Document>();
        // std::cout << "DOC" << std::endl;
        // doc->ParseStream(isw);
        // std::cout << "PARSE!!" << std::endl;
        // if (!doc->IsObject()) {
        //     std::cerr << "Config File: " << filename << " does not exist for JointDynamicAttractor" << std::endl;
        //     exit(-2);
        // }
        // std::cout << "STARTING" << std::endl;
        std::ifstream t(filename);
        // std::cout << "IFSTREAM" << std::endl;
        std::string str((std::istreambuf_iterator<char>(t)),
                        std::istreambuf_iterator<char>());
        t.close();
        // std::cout << "STRING" << std::endl;
        // std::cout << str << std::endl;
        std::string error;
        json = json11::Json::parse(str, error);
        // std::cout << "PARSE" << error << std::endl;
        if (!error.empty()) {
            std::cerr << "Config File: " << filename << " does not exist for JointDynamicAttractor" << std::endl;
            exit(-2);
        }
        // std::cout << "PARSED!!" << std::endl;
    }
};
