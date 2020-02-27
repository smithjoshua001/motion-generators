#pragma once
#include <Eigen/Dense>
#include <DynamicsLib/Computation/RegressorComputation.hpp>
#include <motion-generators/JointTrajectory.hpp>

template <typename T> class RegressorCostFunction : public DL::Computation::RegressorComputation<double> {
private:
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> regressorProjection;
    Eigen::Matrix<T, Eigen::Dynamic, 1> q, qd, qdd;
    std::shared_ptr<DL::util::Pseudoinverse<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> > > pinv;
public:
    RegressorCostFunction(std::shared_ptr<DL::Chain<T> > chain) {
        this->setChain(chain);
        init();
    }
    RegressorCostFunction(std::string model, std::string base, std::string tip) {
        this->loadModel(model);
        this->setChain(base, tip);
        init();
    }
    void init() {
        this->m_dyn->calcBaseProjection(50000, 2);
        // pinv = std::shared_ptr<DL::util::Pseudoinverse<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> > >(new DL::util::Pseudoinverse<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> >(this->m_dyn->getParameterProjector(), 1e-4));
        regressorProjection = this->m_dyn->getRegressorProjector();//pinv->compute(this->m_dyn->getParameterProjector());
        q.resize(this->m_chain->getNumOfJoints());
        qd.resize(this->m_chain->getNumOfJoints());
        qdd.resize(this->m_chain->getNumOfJoints());
        std::cout << this->m_chain->getNumOfJoints() << std::endl;
        std::cout << regressorProjection.rows() << ", " << regressorProjection.cols() << std::endl;
        std::cout << this->out_regressor_var.rows() << ", " << this->out_regressor_var.cols() << std::endl;
        // std::cout << regressorProjection << std::endl;
        // exit(-1);
    }

    int compute(std::shared_ptr<TrajectoryStorage<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic> > input, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &q_out, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &qd_out, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &qdd_out, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &fk_pos_out, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &regressor, bool &collision) {
        assert(input->pos.rows() == this->m_chain.getNumOfJoints());
        assert(fk_pos_out.rows() == 3);
        bool col_output = false;
        // std::cout << "COMPUTING!!" << std::endl;
        // std::cout << input->pos.cols() << std::endl;
        for (size_t i = 0; i < input->pos.cols(); i++) {
            // int i = 4;
            q = input->pos.col(i);//.transpose();
            qd = input->vel.col(i);//.transpose();
            qdd = input->acc.col(i);//.transpose();

            // q.setZero();
            // qd << -0.00, -0.001, -0.0, -0.0, -0.00, 0.0, 0.0;
            // qdd << 0.015, -0.038, -0.022, 0.019, 0.013, 0.006, 0.025;
            this->update(q, qd, true);
            if (!col_output) {
                col_output = this->m_fk->collision();
                // if (col_output) {
                //     std::cout << "COLLIDE" << std::endl;
                // }
            }

            this->computeSlotineLiFriction(qd, qdd);
            // std::cout << this->out_regressor_var.transpose() << std::endl << std::endl;
            regressor.block(i * this->m_chain->getNumOfJoints(), 0, this->m_chain->getNumOfJoints(), this->m_dyn->getNumOfBaseParams()) = this->out_regressor_var * this->m_dyn->getRegressorProjector();
            // std::cout << regressor.block(i * this->m_chain->getNumOfJoints(), 0, this->m_chain->getNumOfJoints(), this->m_dyn->getNumOfBaseParams()).transpose() << std::endl;
            q_out.row(i) = q.transpose();//input->pos.col(i).transpose();
            qd_out.row(i) = qd.transpose();//input->vel.col(i).transpose();
            qdd_out.row(i) = qdd.transpose();//input->acc.col(i).transpose();
            fk_pos_out.row(i) = this->adjoints.back().getP();
        }
        // Eigen::JacobiSVD<Eigen::MatrixXd> svd(regressor);
        // double cond = svd.singularValues()(0)
        //               / svd.singularValues()(svd.singularValues().size() - 1);
        // std::cout << "CONDITION:" << cond << std::endl;

        collision = col_output;
        return (int)col_output;
    }

    size_t getNumOfBaseParams() {
        return this->m_dyn->getNumOfBaseParams();
    }
};
