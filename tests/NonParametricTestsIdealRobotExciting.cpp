// #define CATCH_CONFIG_MAIN
// #include <catch.hpp>

#include "DynamicsLib/Util.h"
#include <DynamicsLib/Model/Chain.h>
#include <DynamicsLib/Kinematics/ForwardKinematicsNew.h>
#include <DynamicsLib/Dynamics/Dynamics.h>
#include <DynamicsLib/Semi-parametric/IGMM.hpp>
#include <DynamicsLib/Semi-parametric/IGMMLinear.hpp>
#include <DynamicsLib/Non-parametric/IGMM.hpp>
#include <DynamicsLib/Semi-parametric/LinearTransform.h>
#include <DynamicsLib/Util.h>
#include <iostream>
#include <fstream>
#include <random>
#include "Iir.h"
#include "motion-generators/ModifiedFourierTrajectory.hpp"

using namespace DL;
int main(int argc, char *argv[]) {
    std::string filename = "";
    std::string fake_filename = "";
    std::string trajectory_config = "";
    double factor = 1;
    double omega = std::numeric_limits<double>::min();
    int tmpj = 1;
    if (argc > 1) {
        filename = std::string(argv[1]);
        if (argc > 2) {
            tmpj = std::atoi(argv[2]);
        }
        if (argc > 3) {
            fake_filename = std::string(argv[3]);
        }
        if (argc > 4) {
            trajectory_config = std::string(argv[4]);
        }
        if (argc > 5) {
            factor = std::atof(argv[5]);
        }
        if (argc > 6) {
            omega = std::atof(argv[6]);
        }
        #ifdef LOG_DEBUG_ON
        if (argc > 7) {
            DL::util::RUNTIME_LOG_LEVEL = std::atoi(argv[7]);
        }
        #endif
    } else {
        return -1;
    }
    std::cout << "D1" << std::endl;
    std::shared_ptr < Chain<double> > c;
    c.reset(new Chain<double>(filename));
    std::shared_ptr < ForwardKinematics<double> > fk;
    fk.reset(new DL::ForwardKinematics<double>(c));
    std::shared_ptr < Dynamics<double> > dyn;
    dyn.reset(new DL::Dynamics<double>(c));

    std::shared_ptr < Chain<double> > c_fake;
    c_fake.reset(new Chain<double>(fake_filename));

    std::cout << "D2" << std::endl;
    Eigen::VectorXd params(c->getNumOfJoints() * 12);
    Eigen::VectorXd fakeparams(c->getNumOfJoints() * 12);
    std::vector<AdjointSE3<double>, Eigen::aligned_allocator<AdjointSE3<double> > > adjoints;
    adjoints.resize(c->getNumOfJoints());
    std::vector<AdjunctSE3<double>, Eigen::aligned_allocator<AdjunctSE3<double> > > adjs(c->getNumOfJoints());
    std::vector<Eigen::Matrix<double, 6, -1>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, -1> > > jacobians, jacobianDots;
    jacobians.resize(c->getNumOfJoints());
    jacobianDots.resize(c->getNumOfJoints());
    for (int i = 0; i < c->getNumOfJoints(); i++) {
        jacobians.at(i).resize(6, c->getNumOfJoints());
        jacobians.at(i).setZero();
        jacobianDots.at(i).resize(6, c->getNumOfJoints());
        jacobianDots.at(i).setZero();
    }
    Eigen::MatrixXd M(c->getNumOfJoints(), c->getNumOfJoints());
    Eigen::MatrixXd C(c->getNumOfJoints(), c->getNumOfJoints());
    Eigen::VectorXd g(c->getNumOfJoints());
    Eigen::VectorXd modelTorque(c->getNumOfJoints());
    Eigen::MatrixXd regressor(c->getNumOfJoints(), c->getNumOfJoints() * 12);

    Eigen::VectorXd f(c->getNumOfJoints());

    std::cout << "D3" << std::endl;
    c->getAllParams(params);
    c_fake->getAllParams(fakeparams);

    ModifiedFourierTrajectory<double> trajectory;
    std::shared_ptr<TrajectoryStorage<double> > traj(new TrajectoryStorage<double>());
    size_t data_no = 0;
    std::string delim = ",";
    size_t pos = 0;
    std::string token;
    while ((pos = trajectory_config.find(delim)) != std::string::npos) {
        token = trajectory_config.substr(0, pos);
        std::cout << token << std::endl;
        trajectory_config.erase(0, pos + delim.length());
        trajectory.loadFromJSON(token);
        std::shared_ptr<TrajectoryStorage<double> > trajTmp = trajectory.simulate(100);
        traj->pos.conservativeResize(c->getNumOfJoints(), traj->pos.cols() + trajTmp->pos.cols());
        traj->vel.conservativeResize(c->getNumOfJoints(), traj->vel.cols() + trajTmp->vel.cols());
        traj->acc.conservativeResize(c->getNumOfJoints(), traj->acc.cols() + trajTmp->acc.cols());

        std::cout << "D4" << std::endl;
        traj->pos.block(0, data_no, c->getNumOfJoints(), std::ceil(trajectory.getPeriodLength() * 100)) = (trajTmp->pos);
        traj->vel.block(0, data_no, c->getNumOfJoints(), std::ceil(trajectory.getPeriodLength() * 100)) = (trajTmp->vel);
        traj->acc.block(0, data_no, c->getNumOfJoints(), std::ceil(trajectory.getPeriodLength() * 100)) = (trajTmp->acc);

        std::cout << "D5" << std::endl;
        data_no += std::ceil(trajectory.getPeriodLength() * 100);

        std::cout << data_no << std::endl;
    }
    trajectory.loadFromJSON(trajectory_config);
    std::shared_ptr<TrajectoryStorage<double> > trajTmp = trajectory.simulate(100);
    traj->pos.conservativeResize(c->getNumOfJoints(), traj->pos.cols() + trajTmp->pos.cols());
    traj->vel.conservativeResize(c->getNumOfJoints(), traj->vel.cols() + trajTmp->vel.cols());
    traj->acc.conservativeResize(c->getNumOfJoints(), traj->acc.cols() + trajTmp->acc.cols());

    std::cout << "D4" << std::endl;
    
    traj->pos.block(0, data_no, c->getNumOfJoints(), std::ceil(trajectory.getPeriodLength() * 100)) = (trajTmp->pos);
    traj->vel.block(0, data_no, c->getNumOfJoints(), std::ceil(trajectory.getPeriodLength() * 100)) = (trajTmp->vel);
    traj->acc.block(0, data_no, c->getNumOfJoints(), std::ceil(trajectory.getPeriodLength() * 100)) = (trajTmp->acc);

    std::cout << "D5" << std::endl;
    data_no += std::ceil(trajectory.getPeriodLength() * 100);

    std::cout << data_no << std::endl;
    std::cout<<data_no<<std::endl;
    exit(-1);
    // return 0;
    Eigen::MatrixXd data(data_no, c->getNumOfJoints() * 4);
    std::cout << "D4" << std::endl;
    std::vector<Eigen::MatrixXd> regressors, fullregressors;
    for (size_t i = 0; i < data_no; i++) {
        data.block(i, 0, 1, c->getNumOfJoints()) = traj->pos.col(i).transpose();
        data.block(i, c->getNumOfJoints(), 1, c->getNumOfJoints()) = traj->vel.col(i).transpose();
        data.block(i, c->getNumOfJoints() * 2, 1, c->getNumOfJoints()) = traj->acc.col(i).transpose();
    }

    std::cout << "D4.1" << std::endl;
    for (size_t i = 0; i < data_no; i++) {
        Eigen::VectorXd q = data.block(i, 0, 1, c->getNumOfJoints()).transpose();
        Eigen::VectorXd qd = data.block(i, c->getNumOfJoints(), 1, c->getNumOfJoints()).transpose();
        Eigen::VectorXd qdd = data.block(i, c->getNumOfJoints() * 2, 1, c->getNumOfJoints()).transpose();

        std::cout << "D4.2" << std::endl;
        c->updateChain(q, qd);
        fk->getAdjoints(adjoints);
        std::cout << "D4.3" << std::endl;
        fk->getBodyJacobian(jacobians, jacobianDots);

        fk->getAdjuncts(adjs, jacobians);
        std::cout << "D4.4" << std::endl;
        dyn->calcJointInertiaMatrix(jacobians, M);

        dyn->calcCoriolisMatrix(jacobians, jacobianDots, adjs, C);
        std::cout << "D4.5" << std::endl;
        dyn->calcGravityVector(jacobians, adjoints, g);

        dyn->calcFrictionVector(f);
        std::cout << "D4.6" << std::endl;
        regressor.setZero();
        dyn->calcSlotineLiRegressor(jacobians, jacobianDots, adjs, adjoints, qd, qdd, regressor);
        Eigen::MatrixXd fullregressor(c->getNumOfJoints() * 4, c->getNumOfJoints() * 12);
        fullregressor.setZero();
        fullregressor.block(c->getNumOfJoints() * 3, 0, c->getNumOfJoints(), c->getNumOfJoints() * 12) = regressor;
        fullregressors.push_back(fullregressor);
        regressors.push_back(regressor);
        std::cout << "D4.7" << std::endl;
        modelTorque = ((M * qdd + C * qd + g + f));
        data.block(i, c->getNumOfJoints() * 3, 1, c->getNumOfJoints()) = modelTorque.transpose();
        std::cout << data.row(i) << std::endl;
        std::cout << modelTorque.transpose() << std::endl;
    }
    std::cout << "D5" << std::endl;
    Eigen::MatrixXd centered_data;
    Eigen::VectorXd stddev(c->getNumOfJoints() * 4);
    std::cout << (data.rowwise() - data.colwise().mean()).colwise().mean() << std::endl;
    centered_data = (data.rowwise() - data.colwise().mean());
    stddev = (centered_data.array().pow(2).colwise().sum() / centered_data.rows()).sqrt();
    std::cout << (centered_data.array().pow(2).colwise().sum() / centered_data.rows()).sqrt() << std::endl;
    // std::sqrt((data.rowwise() - data.mean()).square().sum() / (vec.size() - 1));
    std::cout << "STDDEV!" << std::endl;
    Eigen::VectorXd sigmas(c->getNumOfJoints() * 4);
    std::cout << "FACTOR! " << factor << std::endl;
    sigmas = stddev * factor;
    std::vector<std::pair<double, double> > limits;
    limits.resize(c->getNumOfJoints());

    c->getJointLimits(limits);

    for (size_t i = 0; i < c->getNumOfJoints(); i++) {
        stddev[i] = std::fabs(limits[i].first);
        stddev[i + c->getNumOfJoints()] = 2.5;
        stddev[i + 2 * c->getNumOfJoints()] = 5;
        stddev[i + 3 * c->getNumOfJoints()] = 10;
    }
    std::cout << stddev << std::endl;
    //exit(-1);
    sigmas = stddev * factor;
    // std::cout << stddev.transpose() << std::endl;
    // exit(-1);
    // sigmas.segment(c->getNumOfJoints() * 3, c->getNumOfJoints() - 3).setConstant(1);
    // sigmas.tail(3).setConstant(0.1);
    // sigmas.tail(c->getNumOfJoints()).setConstant(0.1);

    std::cout << "DATA SIZE: " << data.rows() << std::endl;

    // DL::SP::IGMMLinear<double> model(sigmas.size(), 0.0001, sigmas, c->getNumOfJoints() * 12);
    DL::NP::IGMM<double> model(sigmas.size(), omega, sigmas);

    // DL::NP::IGMM<double> model(sigmas.size(), 0.99999, sigmas);
    DL::SP::LinearTransform<double> linTrans(dyn);

    model.setSpLimit(1e5);
    model.setBeta(0.99);
    model.setDetLimit(0);
    std::cout << "PREALLOCATE" << std::endl;
    model.preallocate(50);
    std::cout << "PREALLOCATED!!" << std::endl;
    //auto t1 = std::chrono::high_resolution_clock::now();
    using namespace std::chrono;
    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    const double mean = 0.0;
    const double stddevTmp = 0.1;
    std::random_device rd{};
    std::default_random_engine e1(rd());
    std::normal_distribution<double> dist(mean, stddevTmp);

    for (size_t j = 0; j < tmpj; j++) {
        for (size_t i = 0; i < data.rows(); i = i + 3) {
            Eigen::VectorXd noise;
            noise.resize(c->getNumOfJoints() * 4);
            noise = Eigen::VectorXd::Random(c->getNumOfJoints() * 4) * 0.01;
            Eigen::VectorXd input = data.row(i).transpose();
            input += noise;
            model.incrementalUpdate(input);
        }
        model.print();
    }
    std::cout << "END TRAINING" << std::endl;

    high_resolution_clock::time_point t2 = high_resolution_clock::now();

    duration<double> time_span = duration_cast<duration<double> >(t2 - t1);

    std::cout << "It took me " << time_span.count() / (tmpj * data.rows()) << " seconds.";
    std::cout << std::endl;

    Eigen::MatrixXd before(c->getNumOfJoints(), data.rows());
    for (size_t i = 0; i < data.rows(); i++) {
        model.recall(data.row(i).segment(0, c->getNumOfJoints() * 3).transpose(), before.col(i));
    }

    Eigen::VectorXd paramDot = params * 0.5;    // fakeparams * 0.001;
    // paramDot.setZero();
    // for (size_t i = 0; i < c->getNumOfJoints(); i++) {
    //     paramDot(i * 12) = (params(i * 12) - fakeparams(i * 12));
    // }
    // paramDot.setZero();
    // paramDot.setZero();
    // paramDot(12) = params(12);
    // model.transform(-paramDot);

    Eigen::MatrixXd covDot;
    Eigen::VectorXd muDot;
    covDot = Eigen::MatrixXd::Identity(c->getNumOfJoints() * 4, c->getNumOfJoints() * 4);
    muDot = Eigen::VectorXd::Zero(c->getNumOfJoints() * 4);
    size_t dof = c->getNumOfJoints();
    std::vector<std::shared_ptr<std::pair<DL::NP::MVN<double>, double> > > &components = model.getModels();

    // for (size_t i = 0; i < model.getNumberOfModels(); i++) {
    //     // std::cout << components[i]->first.getMu().transpose() << std::endl << std::endl;
    //     std::cout << 1 / components[i]->first.getInvCov().determinant() << std::endl << std::endl;

    //     std::cout << components[i]->first.getDet() << std::endl << std::endl;
    // }

    t1 = high_resolution_clock::now();
    double tmpi0 = 20;
    for (size_t i0 = 0; i0 < tmpi0; i0++) {
        for (size_t i = 0; i < model.getNumberOfModels(); i++) {
            // covDot.setIdentity();
            covDot.setZero();
            linTrans.calcGMMTransforms(components[i]->first.getMu(), -paramDot / tmpi0, muDot.segment(3 * dof, dof), covDot.block(dof * 3, 0, dof, dof * 3));
            // covDot.block(0, c->getNumOfJoints() * 3, c->getNumOfJoints() * 3, c->getNumOfJoints()) = covDot.block(dof * 3, 0, dof, dof * 3).transpose();
            // std::cout << covDot << std::endl;
            // covDot.transposeInPlace();
            components[i]->first.transformTesting(muDot, covDot);

            // components[i]->first.transform(muDot, covDot);
        }
    }
    t2 = high_resolution_clock::now();
    time_span = duration_cast<duration<double> >(t2 - t1);

    std::cout << "It took me " << time_span.count() << " seconds.";
    std::cout << std::endl;
    std::cout << "It took me " << time_span.count() / model.getNumberOfModels() << " seconds.";
    std::cout << std::endl;

    //auto t2 = std::chrono::high_resolution_clock::now();
    //std::chrono::duration<double, std::milli> fp_ms = t2 - t1;
    //std::cout << "TOOK: " << fp_ms.count() / data.rows()
    //          << " milliseconds\n";
    // for (size_t i = 0; i < data.rows(); i++) {
    //     if (std::fmod(((double)i / (double)data.rows()), 0.2) <= 1e-5) {
    //         std::cout << i << std::endl;
    //         std::cout << ((double)i / (double)data.rows()) << "% " << std::fmod(((double)i / (double)data.rows()), 0.2) << std::endl;
    //     }
    //     //std::cout << data.row(i) << std::endl;
    //     model.incrementalUpdate(data.row(i).transpose());
    // }
    std::cout << "Finish!!" << std::endl;
    /*model.print();
    Eigen::VectorXd temp(2);
    Eigen::VectorXd acc(2);
    // float acc = 0;
    acc.setZero();*/
    std::ofstream ofs("RSE.dat", std::ofstream::out);
    ofs << "index ";
    for (size_t i = 0; i < data.cols() / 4; i++) {
        ofs << "joint_" << i << "_real ";
    }
    for (size_t i = 0; i < data.cols() / 4; i++) {
        ofs << "joint_" << i << "_recall ";
    }
    for (size_t i = 0; i < data.cols() / 4; i++) {
        ofs << "joint_" << i << "_real_before ";
    }
    for (size_t i = 0; i < data.cols() / 4; i++) {
        ofs << "joint_" << i << "_recall_before ";
    }
    ofs << std::endl;

    // for (size_t j = 0; j < tmpj; j++) {
    //     for (size_t i = 0; i < data.rows(); i = i + 1) {
    //         // if (std::fmod(((double)i / (double)filteredData.rows()), 0.2) <= 2e-5) {
    //         //std::cout << i << std::endl;
    //         //std::cout << ((double)i / (double)filteredData.rows()) << "% " << std::fmod(((double)i / (double)filteredData.rows()), 0.2) << std::endl;
    //         //model.print();
    //         // }
    //         //std::cout << data.row(i) << std::endl;
    //         //model.print();
    //         // std::cout << fullregressors[i] << std::endl;
    //         Eigen::VectorXd noise;
    //         noise.resize(c->getNumOfJoints() * 4);
    //         noise = Eigen::VectorXd::Random(c->getNumOfJoints() * 4) * 0.1;
    //         Eigen::VectorXd input = data.row(i).transpose();
    //         Eigen::VectorXd tmp3 = (data.row(i).segment(c->getNumOfJoints() * 3, c->getNumOfJoints())).transpose();
    //         tmp3 -= regressors[i] * paramDot;
    //         input.tail(c->getNumOfJoints()) = tmp3;
    //         // input.tail(c->getNumOfJoints()) += noise;
    //         input += noise;
    //         model.incrementalUpdate(input);
    //         // model.print();
    //     }
    //     model.print();
    // }

    Eigen::VectorXd temp(c->getNumOfJoints());
    Eigen::ArrayXd acc(c->getNumOfJoints());
    Eigen::ArrayXd acc2(c->getNumOfJoints());
    acc.setZero();
    acc2.setZero();
    for (size_t j = 0; j < 1; j++) {
        for (size_t i = 0; i < data.rows(); i++) {
            // std::cout << i << std::endl;
            // model.incrementalUpdate(data.row(i).transpose());
            model.recall(data.row(i).segment(0, c->getNumOfJoints() * 3).transpose(), temp);
            //std::cout << temp.transpose() << std::endl;
            // std::cout << temp.transpose() - data.row(i).segment(6, 2) << std::endl;
            // std::cout << "================================" << std::endl;

            Eigen::VectorXd tmp3 = (data.row(i).segment(c->getNumOfJoints() * 3, c->getNumOfJoints())).transpose();
            tmp3 -= regressors[i] * paramDot;
            acc += (temp - tmp3).array().abs().pow(2) / (1 * data.rows());
            acc2 += (before.col(i) - tmp3).array().abs().pow(2) / (1 * data.rows());
            Eigen::VectorXd tmp4 = (data.row(i).segment(c->getNumOfJoints() * 3, c->getNumOfJoints())).transpose();
            Eigen::VectorXd tmp5 = before.col(i);
            ofs << i + j * data.rows() << " " << tmp3.transpose() << " " << temp.transpose() << " " << tmp4.transpose() << " " << tmp5.transpose() << std::endl;
            // std::cout << i << std::endl;
        }
    }
    // acc = acc / (10 * data.rows());
    std::cout << acc.sqrt() << std::endl << std::endl;
    std::cout << acc2.sqrt() << std::endl;
    // std::cout << "Param Error" << std::endl << params - fakeparams << std::endl;
    std::cout << "Param Dot" << std::endl << paramDot.transpose() << std::endl << std::endl;
    ofs.close();    /*
    model.save("igmm.igmm");*/
    for (size_t i = 0; i < model.getNumberOfModels(); i++) {
        // std::cout << components[i]->first.getMu().transpose() << std::endl << std::endl;
        std::cout << 1 / components[i]->first.getInvCov().determinant() << std::endl << std::endl;
        std::cout << components[i]->first.getDet() << std::endl << std::endl;
    }
    return 1;
}
