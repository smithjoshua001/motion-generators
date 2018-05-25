#pragma once
#include <Eigen/Dense>
#include <motion-generators/JointTrajectory.hpp>
#include <random>
#include <cmath>
#include <iostream>
#include <Eigen/StdVector>

template <typename T> class PulsatingJointTrajectory : public JointTrajectory<T> {
// public:
//     typedef std::vector<Eigen::Matrix<T,Eigen::Dynamic,1>,Eigen::aligned_allocator<Eigen::Matrix<T,Eigen::Dynamic,1>>> vector_aa;
private:
    T global_pulsation;
    size_t fourier_coeff_number;
    // vector_aa fourier_coeff_a;
    // vector_aa fourier_coeff_b;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> fourier_coeff_a, fourier_coeff_b;
    Eigen::Matrix<T, Eigen::Dynamic, 1> joint_offsets;

    std::random_device rd;
    std::mt19937 gen;
    std::uniform_real_distribution<T> real_dist;
public:
    PulsatingJointTrajectory(size_t dof) : JointTrajectory<T>(dof) {
        gen = std::mt19937(rd());
        real_dist = std::uniform_real_distribution<T>(0, std::nextafter(1, std::numeric_limits<T>::max()));
        global_pulsation = 1.0;
        fourier_coeff_number = 4;
        joint_offsets.resize(dof);
        fourier_coeff_a.resize(dof, fourier_coeff_number);
        fourier_coeff_b.resize(dof, fourier_coeff_number);
        for (size_t i = 0; i < dof; i++) {
            joint_offsets[i] = real_dist(gen) * 2 - 1;
            for (size_t j = 0; j < fourier_coeff_number; j++) {
                T maximum = 2 - std::fabs(joint_offsets[i]);
                fourier_coeff_a(i, j) = real_dist(gen) * maximum - maximum / 2;
                fourier_coeff_b(i, j) = real_dist(gen) * maximum - maximum / 2;
            }
        }
    }
    ~PulsatingJointTrajectory() override {}
    // Eigen::Matrix<T,Eigen::Dynamic,1>const& getPosition(double time=0) override{
    //     this->position.setZero();
    //     //for(size_t i = 0; i < this->dof; i++){
    //         for(size_t j = 1; j<= fourier_coeff_number;j++){
    //             this->position += (fourier_coeff_a.col(j-1)/(global_pulsation*j))*std::sin(global_pulsation*j*time) -
    //                 (fourier_coeff_b.col(j-1)/(global_pulsation*j))*std::cos(global_pulsation*j*time);
    //         }
    //         this->position+=fourier_coeff_number*joint_offsets;
    //     //}
    //     return this->position;
    // }
    // Eigen::Matrix<T,Eigen::Dynamic,1>const& getVelocity(double time=0) override{
    //     this->velocity.setZero();
    //     for(size_t i = 0; i < this->dof; i++){
    //         for(size_t j = 1; j<= fourier_coeff_number;j++){
    //             this->velocity[i] += (fourier_coeff_a(i,j-1))*std::cos(global_pulsation*j*time) +
    //                 (fourier_coeff_b(i,j-1))*std::sin(global_pulsation*j*time);
    //         }
    //     }
    //     return this->velocity;
    // }
    // Eigen::Matrix<T,Eigen::Dynamic,1>const& getAcceleration(double time=0) override{
    //     this->acceleration.setZero();
    //     for(size_t i = 0; i < this->dof; i++){
    //         for(size_t j = 1; j<= fourier_coeff_number;j++){
    //             this->acceleration[i] += -(fourier_coeff_a(i,j-1)*(global_pulsation*j))*std::sin(global_pulsation*j*time) +
    //                 (fourier_coeff_b(i,j-1)*(global_pulsation*j))*std::cos(global_pulsation*j*time);
    //         }
    //     }
    //     return this->acceleration;
    // }
    void update(double time = 0) override {
        this->position.setZero();
        for (size_t j = 1; j <= fourier_coeff_number; j++) {
            this->position += (fourier_coeff_a.col(j - 1) / (global_pulsation * j)) * std::sin(global_pulsation * j * time) -
                              (fourier_coeff_b.col(j - 1) / (global_pulsation * j)) * std::cos(global_pulsation * j * time);
        }
        this->position += fourier_coeff_number * joint_offsets;
        this->velocity.setZero();
        for (size_t i = 0; i < this->dof; i++) {
            for (size_t j = 1; j <= fourier_coeff_number; j++) {
                this->velocity[i] += (fourier_coeff_a(i, j - 1)) * std::cos(global_pulsation * j * time) +
                                     (fourier_coeff_b(i, j - 1)) * std::sin(global_pulsation * j * time);
            }
        }
        this->acceleration.setZero();
        for (size_t i = 0; i < this->dof; i++) {
            for (size_t j = 1; j <= fourier_coeff_number; j++) {
                this->acceleration[i] += -(fourier_coeff_a(i, j - 1) * (global_pulsation * j)) * std::sin(global_pulsation * j * time) +
                                         (fourier_coeff_b(i, j - 1) * (global_pulsation * j)) * std::cos(global_pulsation * j * time);
            }
        }
    }
    void setGlobalPulsation(T global_pulse) {
        global_pulsation = global_pulse;
    }
    void setCoefficentNumbers(size_t fourier_coeff_number) {
        this->fourier_coeff_number = fourier_coeff_number;
    }
    // void setFourierCoefficientsA(vector_aa fourier_coeff_a){
    //     this->fourier_coeff_a = fourier_coeff_a;
    // }
    // vector_aa& getFourierCoefficientsA(){
    //     return fourier_coeff_a;
    // }
    // void setFourierCoefficientsB(vector_aa fourier_coeff_b){
    //     this->fourier_coeff_b = fourier_coeff_b;
    // }
    // vector_aa& getFourierCoefficientsB(){
    //     return fourier_coeff_b;
    // }
    void setJointOffsets(Eigen::Matrix<T, Eigen::Dynamic, 1> joint_offsets) {
        this->joint_offsets = joint_offsets;
    }
    void display() {
        std::cout << "PJT\n";
        std::cout << "Global Pulsation:" << global_pulsation << "\n";
        std::cout << "coefficients numbers";
        std::cout << fourier_coeff_number;
        std::cout << "\n";
        std::cout << "coefficients a:";
        for (size_t i = 0; i < this->dof; i++) {
            std::cout << fourier_coeff_a << "\n ";
        }
        std::cout << "\n";
        std::cout << "coefficients b:";
        for (size_t i = 0; i < this->dof; i++) {
            std::cout << fourier_coeff_b << "\n ";
        }
        std::cout << "\n";
    }
    T getPeriodLength() {
        return 2 * M_PI / this->global_pulsation;
    }
};
