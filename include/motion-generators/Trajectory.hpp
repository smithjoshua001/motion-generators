#pragma once
#include <Eigen/Dense>
#include <type_traits>
#include <fstream>
#include <memory>
#include <string>
#include <streambuf>
#include <iostream>
#include <json11.hpp>

template <typename T, int pos_dim = Eigen::Dynamic, int vel_dim = Eigen::Dynamic, int acc_dim = vel_dim> struct TrajectoryStorage {
    Eigen::Matrix<T, pos_dim, Eigen::Dynamic> pos;
    Eigen::Matrix<T, vel_dim, Eigen::Dynamic> vel;
    Eigen::Matrix<T, acc_dim, Eigen::Dynamic> acc;
    TrajectoryStorage(Eigen::Matrix<T, pos_dim, Eigen::Dynamic> pos, Eigen::Matrix<T, vel_dim, Eigen::Dynamic> vel, Eigen::Matrix<T, acc_dim, Eigen::Dynamic> acc) {
        this->pos = pos;
        this->vel = vel;
        this->acc = acc;
    }
    TrajectoryStorage() {}
    ~TrajectoryStorage() {}

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

template <typename T, int pos_dim = Eigen::Dynamic, int vel_dim = Eigen::Dynamic, int acc_dim = Eigen::Dynamic> class Trajectory {
protected:
    Eigen::Matrix<T, pos_dim, 1> position;
    Eigen::Matrix<T, vel_dim, 1> velocity;
    Eigen::Matrix<T, acc_dim, 1> acceleration;
    std::shared_ptr<TrajectoryStorage<T, pos_dim, vel_dim, acc_dim> > trajectory;
    bool runnable;
    json11::Json json;
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Trajectory() {
        runnable = false;
        trajectory = std::shared_ptr<TrajectoryStorage<T, pos_dim, vel_dim, acc_dim> >(new TrajectoryStorage<T, pos_dim, vel_dim, acc_dim>());
    }

    virtual ~Trajectory() {};
    virtual void update(double dt = 0) = 0;
    Eigen::Matrix<T, pos_dim, 1> const &getPosition() {return position;}
    Eigen::Matrix<T, vel_dim, 1> const &getVelocity() {return velocity;}
    Eigen::Matrix<T, acc_dim, 1> const &getAcceleration() {return acceleration;}

    virtual T getPeriodLength() = 0;// {return T(0);}

    virtual std::shared_ptr<TrajectoryStorage<T, pos_dim, vel_dim, acc_dim> > simulate(T control_frequency) = 0;

    bool &getRunnable() {
        return runnable;
    }

    std::shared_ptr<TrajectoryStorage<T, pos_dim, vel_dim, acc_dim> > getTrajectoryStorage() {
        return trajectory;
    }

    virtual void loadFromJSON(std::string filename) {
        std::ifstream t(filename);
        std::string str((std::istreambuf_iterator<char>(t)),
                        std::istreambuf_iterator<char>());
        t.close();
        std::string error;
        json = json11::Json::parse(str, error);
        if (!error.empty()) {
            std::cerr << "Config File: " << filename << " does not exist for JointDynamicAttractor" << std::endl;
            exit(-2);
        }
    }
};
