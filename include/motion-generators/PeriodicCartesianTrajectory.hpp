#pragma once
#include "motion-generators/CartesianTrajectory.hpp"
namespace motion_generators {
    namespace taskspace {
        template <typename T> class PeriodicCartesianTrajectory : public CartesianTrajectory<T> {
        public:
            PeriodicCartesianTrajectory() : CartesianTrajectory<T>() {
                this->position.setZero();
                this->velocity.setZero();
                this->acceleration.setZero();
            }

            PeriodicCartesianTrajectory(Eigen::Matrix<T, 7, 1> position) : CartesianTrajectory<T>(position) {}
            virtual ~PeriodicCartesianTrajectory() {};

            TrajectoryStorage<T, 7, 6, 6> simulate(T control_frequency) override {
                assert(runnable);
                //TODO better solution
                assert(this->getPeriodLength() > 0 && "Invalid Trajectory to simulate");
                int simulated_points = std::ceil(this->getPeriodLength() * control_frequency);
                // Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> pos, vel, acc;
                this->trajectory.pos.resize(7, simulated_points);
                this->trajectory.vel.resize(6, simulated_points);
                this->trajectory.acc.resize(6, simulated_points);
                for (int i = 0; i < simulated_points; i++) {
                    this->update(1 / control_frequency);
                    this->trajectory.pos.col(i) = this->getPosition();
                    this->trajectory.vel.col(i) = this->getVelocity();
                    this->trajectory.acc.col(i) = this->getAcceleration();
                }
                return this->trajectory;
            }
        };
    }
}
