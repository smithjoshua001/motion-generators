#pragma once
#include <Eigen/Dense>
#include <type_traits>

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
};

template <typename T, int pos_dim = Eigen::Dynamic, int vel_dim = Eigen::Dynamic, int acc_dim = Eigen::Dynamic> class Trajectory {
protected:
    Eigen::Matrix<T, pos_dim, 1> position;
    Eigen::Matrix<T, vel_dim, 1> velocity;
    Eigen::Matrix<T, acc_dim, 1> acceleration;
    TrajectoryStorage<T, pos_dim, vel_dim, acc_dim> trajectory;
    bool runnable;
    json11::Json json;
public:

    Trajectory() {
        runnable = false;
    }

    virtual ~Trajectory() {};
    virtual void update(double dt = 0) = 0;
    Eigen::Matrix<T, pos_dim, 1> const &getPosition() {return position;}
    Eigen::Matrix<T, vel_dim, 1> const &getVelocity() {return velocity;}
    Eigen::Matrix<T, acc_dim, 1> const &getAcceleration() {return acceleration;}

    T getPeriodLength() {return T(0);}

    virtual TrajectoryStorage<T, pos_dim, vel_dim, acc_dim> simulate(T control_frequency) = 0;

    bool &getRunnable() {
        return runnable;
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
