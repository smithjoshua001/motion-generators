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

// template <typename T> struct Trajectory {
//     Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> pos, vel, acc;
//     Trajectory(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> pos, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> vel, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> acc) {
//         this->pos = pos;
//         this->vel = vel;
//         this->acc = acc;
//     }
//     Trajectory() {}
// };

#include <fstream>
#include <memory>
#include <string>
#include <streambuf>
#include <json11.hpp>

#include "motion-generators/Trajectory.hpp"

template <typename T, int DOF_C = -1> class JointTrajectory : public Trajectory<T, DOF_C, DOF_C, DOF_C> {
protected:
    size_t dof;
// Eigen::Matrix<T, Eigen::Dynamic, 1> position;
// Eigen::Matrix<T, Eigen::Dynamic, 1> velocity;
// Eigen::Matrix<T, Eigen::Dynamic, 1> acceleration;
// TrajectoryStorage<T> trajectory;
// rapidjson::GenericDocument<rapidjson::UTF8<> > doc;
// json11::Json json;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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

    TrajectoryStorage<T> simulate(T control_frequency) {
        assert(this->runnable);
        //TODO better solution
        assert(this->getPeriodLength() > 0 && "Invalid Trajectory to simulate");
        int simulated_points = std::ceil(this->getPeriodLength() * control_frequency);
        // Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> pos, vel, acc;
        this->trajectory.pos.resize(this->dof, simulated_points);
        this->trajectory.vel.resize(this->dof, simulated_points);
        this->trajectory.acc.resize(this->dof, simulated_points);
        for (int i = 0; i < simulated_points; i++) {
            // std::cout << ((T)i) / control_frequency << "\n";
            this->update(((T)i) / control_frequency);
            this->trajectory.pos.col(i) = this->getPosition();
            this->trajectory.vel.col(i) = this->getVelocity();
            this->trajectory.acc.col(i) = this->getAcceleration();
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
        this->json = json11::Json::parse(str, error);
        // std::cout << "PARSE" << error << std::endl;
        if (!error.empty()) {
            std::cerr << "Config File: " << filename << " does not exist for JointDynamicAttractor" << std::endl;
            exit(-2);
        }
        // std::cout << "PARSED!!" << std::endl;
    }
};
