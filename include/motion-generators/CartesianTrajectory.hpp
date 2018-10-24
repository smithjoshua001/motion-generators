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
namespace motion_generators {
    namespace taskspace {
        template <typename T> class CartesianTrajectory : public Trajectory<T, 7, 6, 6> {
        public:
            CartesianTrajectory() : Trajectory<T, 7, 6, 6>() {
                this->position.setZero();
                this->velocity.setZero();
                this->acceleration.setZero();
            }

            CartesianTrajectory(Eigen::Matrix<T, 7, 1> position) {
                this->position = position;
                this->velocity.setZero();
                this->acceleration.setZero();
                this->runnable = true;
            }
            virtual ~CartesianTrajectory() {};

            TrajectoryStorage<T, 7, 6, 6> simulate(T control_frequency) override {
                return this->trajectory;
            }

            // template <bool p = periodic > typename std::enable_if< p, TrajectoryStorage<T, 7, 6, 6> >::type simulateImpl(T control_frequency) {
            //     assert(runnable);
            //     //TODO better solution
            //     assert(this->getPeriodLength() > 0 && "Invalid Trajectory to simulate");
            //     int simulated_points = std::ceil(this->getPeriodLength() * control_frequency);
            //     // Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> pos, vel, acc;
            //     this->trajectory.pos.resize(7, simulated_points);
            //     this->trajectory.vel.resize(6, simulated_points);
            //     this->trajectory.acc.resize(6, simulated_points);
            //     for (int i = 0; i < simulated_points; i++) {
            //         this->update(1 / control_frequency);
            //         this->trajectory.pos.col(i) = this->getPosition();
            //         this->trajectory.vel.col(i) = this->getVelocity();
            //         this->trajectory.acc.col(i) = this->getAcceleration();
            //     }
            //     return this->trajectory;
            // }
            // template <bool p = periodic > typename std::enable_if< !p, TrajectoryStorage<T, 7, 6, 6> >::type simulateImpl(T control_frequency) {
            //     return this->trajectory;
            // };

            // std::enable_if_t<periodic, TrajectoryStorage<T, 7, 6, 6> > simulate(T control_frequency) = 0;
            virtual void loadFromJSON(std::string filename) {
                Trajectory<T, 7, 6, 6>::loadFromJSON(filename);
            }
        };
    }
}
