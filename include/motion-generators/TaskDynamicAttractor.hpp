#pragma once
#include <Eigen/Dense>
#include <string>
#include "motion-generators/CartesianTrajectory.hpp"

namespace motion_generators {
    namespace taskspace {
        class TaskDynamicAttractor : public CartesianTrajectory<double> {
        public:
            TaskDynamicAttractor() {
                first_config = false;
                go = false;

                state.resize(13);
                state.setZero();
                state_dot.resize(13);
                state_dot.setZero();

                des_state.resize(13);
                des_state.setZero();
                des_state_dot.resize(13);
                des_state_dot.setZero();

                gainMatrix.resize(6, 13);
                gainMatrix.setZero();

                state_error.resize(13);
                state_error.setZero();
                quat_left.resize(4, 4);
                quat_left.setZero();
            }
            void setStartTime(const double &start_time) {
                this->start_time = start_time;
            };

            void update(double dt) override {
                if (first_config && go) {
                    computeError();
                    Eigen::Quaternionf temp(des_state(3), des_state(4), des_state(5), des_state(6));
                    Eigen::Quaternionf temp2(state(3), state(4), state(5), state(6));
                    Eigen::Quaternionf temp3 = temp2 * temp.inverse();//temp.inverse()*temp2;
                    temp3.normalize();

                    quat_left.block<3, 3>(1, 1) = skewMat + des_state(3) * Eigen::Matrix3d::Identity();
                    quat_left.block<3, 1>(1, 0) = des_state.segment<3>(4);
                    quat_left.block<1, 3>(0, 1) = -des_state.segment<3>(4).transpose();
                    quat_left(0, 0) = des_state(3);

                    Eigen::MatrixXd omega_mat(4, 4);
                    omega_mat.setZero();
                    skewMatrix(state.segment<3>(10), skewMat);
                    omega_mat.block<3, 3>(1, 1) = -skewMat;
                    omega_mat.block<3, 1>(1, 0) = state.segment<3>(10);
                    omega_mat.block<1, 3>(0, 1) = -state.segment<3>(10).transpose();
                    state_dot.segment<4>(3) = (0.5 * (quat_left * omega_mat) * (state_error.segment<4>(3)));
                    state_dot.segment<3>(0) = state_error.segment<3>(7);

                    Eigen::Quaternionf temp4(0, state(10), state(11), state(12));
                    Eigen::Quaternionf temp5(state_error(3), state_error(4), state_error(5), state_error(6));
                    Eigen::Quaternionf temp6 = (temp4) * (temp5 * temp);
                    //state_dot.segment<4>(3) = 0.5*;
                    state_dot(3) = 0.5 * temp6.w();
                    state_dot(4) = 0.5 * temp6.x();
                    state_dot(5) = 0.5 * temp6.y();
                    state_dot(6) = 0.5 * temp6.z();

                    state_dot.tail<6>() = gainMatrix * state_error;
                    state = state + state_dot * dt;

                    if (state_error.norm() - 1 < 0.0001) {
                        reset();
                    }
                }
            }

            virtual void reset() {
                first_config = false;
                go = false;
            }

            void setInitialState(const Eigen::VectorXd &initialState) {
                state = initialState;
                first_config = true;
            }

            void setDesiredPose(const Eigen::VectorXd &desiredState) {
                des_state = desiredState;
                computeError();
                maxError = state_error.head<7>().norm() - 1;
                go = true;
            }

            template<typename Derived, typename Derived2> static void skewMatrix(const Eigen::MatrixBase<Derived> &vector, Eigen::MatrixBase<Derived2> &skewMat) {
                skewMat.setZero();
                skewMat(2, 1) = vector(0);
                skewMat(1, 2) = -vector(0);
                skewMat(0, 2) = vector(1);
                skewMat(2, 0) = -vector(1);
                skewMat(0, 1) = -vector(2);
                skewMat(1, 0) = vector(2);
            }

            void computeError() {
                if (state.segment<4>(3).transpose() * des_state.segment<4>(3) < 0) {
                    des_state.segment<4>(3) = -des_state.segment<4>(3);
                }
                skewMatrix(des_state.segment<3>(4), skewMat);
                quat_left.block<3, 3>(1, 1) = -skewMat + des_state(3) * Eigen::Matrix3d::Identity();
                quat_left.block<3, 1>(1, 0) = -des_state.segment<3>(4);
                quat_left.block<1, 3>(0, 1) = des_state.segment<3>(4).transpose();
                quat_left(0, 0) = des_state(3);
                state_error = state - des_state;
                state_error.segment<4>(3) = quat_left * state.segment<4>(3);

                Eigen::Quaternionf temp(des_state(3), des_state(4), des_state(5), des_state(6));
                Eigen::Quaternionf temp2(state(3), state(4), state(5), state(6));
                Eigen::Quaternionf temp3 = temp2 * temp.inverse();//temp.inverse()*temp2;
                temp3.normalize();
                //std::cout<<state_error.segment<4>(3).transpose()<<" : "<<temp3.w()<<", "<<temp3.x()<<", "<<temp3.y()<<", "<<temp3.z()<<"\n";

                state_error(3) = temp3.w();
                state_error(4) = temp3.x();
                state_error(5) = temp3.y();
                state_error(6) = temp3.z();
            }

            void setGains(const double &gain, const double &orientation_gain) {
                gainMatrix.block(0, 0, 3, 3).setIdentity();
                gainMatrix.block(3, 4, 3, 3).setIdentity();
                gainMatrix.block(3, 4, 3, 3) *= -orientation_gain * orientation_gain;

                gainMatrix.block(0, 0, 3, 3) *= -gain * gain;
                gainMatrix.block(0, 7, 6, 6).setIdentity();
                gainMatrix.block(0, 7, 3, 3) *= -2 * gain;
                gainMatrix.block(3, 10, 3, 3) *= -2 * orientation_gain;
            }

            void loadFromJSON(std::string filename) override {
                CartesianTrajectory<double>::loadFromJSON(filename);
                this->setGains(json["gain"].number_value(), json["orientaion_gain"].number_value());
            }

        protected:
            Eigen::MatrixXd gainMatrix;
            Eigen::MatrixXd skewGainMatrix;
            Eigen::Matrix3d skewMat;
            Eigen::VectorXd state, state_dot;
            Eigen::VectorXd des_state, des_state_dot;

            Eigen::VectorXd state_error;
            Eigen::MatrixXd quat_left;

            bool portsArePrepared, first_config, go;

            double maxError;

            double start_time;
        };
    }
}
