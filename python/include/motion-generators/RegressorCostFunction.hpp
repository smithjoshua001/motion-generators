#pragma once
#include <Eigen/Dense>
#include <DynamicsLib/AdjointTransform.h>
#include <DynamicsLib/Chain.h>
#include <DynamicsLib/Dynamics/ForwardDynamics.h>
#include <DynamicsLib/Kinematics/ForwardKinematics.h>
#include <motion-generators/JointTrajectory.hpp>

template <typename T> class RegressorCostFunction {
private:
    std::shared_ptr<DL::Chain<T> > chain;
    std::unique_ptr<DL::ForwardKinematics<T> > fk;
    std::unique_ptr<DL::ForwardDynamics<T> > dyn;
    std::vector<DL::AdjointTransform<T> > inverseAdjoints;
    std::vector<Eigen::Matrix<T, 6, -1>, Eigen::aligned_allocator<Eigen::Matrix<T, 6, -1> > > jacobians, jacobianDots;
    DL::AdjointTransform<T> tempAd;
    std::vector<Eigen::Matrix<T, 6, 6>, Eigen::aligned_allocator<Eigen::Matrix<T, 6, 6> > > adjs;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> regressorProjection;
    Eigen::Matrix<T, Eigen::Dynamic, 1> q, qd, qdd;
public:
    RegressorCostFunction(std::shared_ptr<DL::Chain<T> > chain) {
        this->chain = chain;
        fk = std::make_unique<DL::ForwardKinematics<T> >(chain);
        dyn = std::make_unique<DL::ForwardDynamics<T> >(chain);

        inverseAdjoints.resize(chain->getNumOfJoints());
        jacobians.resize(chain->getNumOfJoints());
        jacobianDots.resize(chain->getNumOfJoints());
        for (int i = 0; i < chain->getNumOfJoints(); i++) {
            jacobians.at(i).resize(6, chain->getNumOfJoints());
            jacobians.at(i).setZero();
            jacobianDots.at(i).resize(6, chain->getNumOfJoints());
            jacobianDots.at(i).setZero();
        }
        adjs.resize(chain->getNumOfJoints());
        dyn->calcBaseProjection(2000, 5);
        regressorProjection = dyn->getRegressorProjector();
        q.resize(chain->getNumOfJoints());
        qd.resize(chain->getNumOfJoints());
        qdd.resize(chain->getNumOfJoints());
    }

    void compute(Trajectory<T> input, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &q_out, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &qd_out, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &qdd_out, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &fk_pos_out, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &regressor, bool &collision) {
        // Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> regressor;
        // regressor.resize(input.pos.rows() * chain->getNumOfJoints(), dyn->getNumOfBaseParams());
        // regressor.setZero();
        assert(input.pos.rows() == chain.getNumOfJoints());
        assert(fk_pos_out.rows() == 3);
        LOG_DEBUG_LEVEL5(DL::util::console, "TESTING: {} {}", input.pos.rows(), input.pos.cols());
        for (size_t i = 0; i < input.pos.cols(); i++) {
            q = input.pos.col(i).transpose();
            qd = input.vel.col(i).transpose();
            qdd = input.acc.col(i).transpose();
            LOG_DEBUG_LEVEL5(DL::util::console, "TESTING: {}", q.transpose());
            chain->updateChainPos(q);
            chain->updateChainVel(qd);

            fk->getInverseAdjoints(inverseAdjoints, true);
            collision = collision || fk->collision();

            fk->getSpatialJacobianPropagationAd(q, qd, jacobians, jacobianDots, false);

            fk->getAdjugates(qd, jacobians, adjs);

            dyn->calcSlotineLiBaseRegressor(dyn->getRegressorProjector(), jacobians, jacobianDots, adjs, inverseAdjoints, qd, qdd, regressor.block(i * chain->getNumOfJoints(), 0, chain->getNumOfJoints(), dyn->getNumOfBaseParams()));
            q_out.row(i) = input.pos.col(i).transpose();
            qd_out.row(i) = input.vel.col(i).transpose();
            qdd_out.row(i) = input.acc.col(i).transpose();
            fk_pos_out.row(i) = inverseAdjoints.back().getP();
        }
    }

    size_t getNumOfBaseParams() {
        return dyn->getNumOfBaseParams();
    }

    // self.FK.getInverseAdjoints(self.adjv, True)
    //         collision = collision or self.FK.collision()
    //         self.FK.getSpatialJacobianPropagationAd(self.jacv, self.jacdv)
    //         self.FK.getSpatialJacobianPropagationAd(
    //             q, qd, self.jacv, self.jacdv, False)
    //         # self.regressor2[k, :, :]) #
    //         self.FK.getAdjugates(qd, self.jacv, self.av)
    //         self.DYN.calcSlotineLiBaseRegressor(self.temp, self.jacv, self.jacdv, self.av, self.adjv, qd, qdd, self.regressor[k*(
    //             self.dof): (k+1)*(self.dof), 0: self.DYN.getNumOfBaseParams()])
};
