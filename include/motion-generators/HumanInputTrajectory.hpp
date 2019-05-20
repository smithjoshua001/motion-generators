#pragma once
#include <Eigen/Dense>
#include <string>
#include "motion-generators/CartesianTrajectory.hpp"
namespace motion_generators {
    namespace taskspace {
        template <typename T> class HumanInputTrajectory : public CartesianTrajectory<T>{
        protected:
            Eigen::Matrix<T, 3, 1> errorPosition;
            Eigen::Matrix<T, 7, 1> currentPose, desiredPose;
            T maxDistance;
        public:
            HumanInputTrajectory() : CartesianTrajectory<T>() {
                currentPose.setZero();
                desiredPose.setZero();
                desiredPose[3]=1;
                errorPosition.setZero();
                maxDistance = 0.1;
            }
            ~HumanInputTrajectory() {}
            void loadFromJSON(std::string filename) override {
                CartesianTrajectory<T>::loadFromJSON(filename);
            }

            void update(double dt) override {
                //check and shrink distance (position only first)
                errorPosition = desiredPose.template head<3>() - currentPose.template head<3>();
                if (errorPosition.norm() > maxDistance) {
                    desiredPose.template head<3>() = currentPose.template head<3>() + (errorPosition * (maxDistance / errorPosition.norm()));
                }
                //TODO add orientation limits
            }

            template <typename Derived> void setCurrentPose(const Eigen::MatrixBase<Derived> &pose) {
                currentPose = pose;
            }

            template <typename Derived> void setDesiredPose(const Eigen::MatrixBase<Derived> &pose) {
                desiredPose = pose;
            }

            virtual void reset() {};
        };
    }
}
