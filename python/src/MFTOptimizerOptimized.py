import DynamicsLib
import numpy as np
import ModifiedFourierTrajectory
import OptimizerBase
import pyOpt
import RegressorCostFunction

from math import ceil
import sys


class ModifiedFourierTrajectoryOptimzerOptim(OptimizerBase.OptimizerBase):

    def __init__(self, TrajModel, Chain, config):
        super(ModifiedFourierTrajectoryOptimzerOptim, self).__init__(config)
        print('Debug 1')
        self.TrajModel = TrajModel
        self.Chain = Chain

        self.kin = DynamicsLib.ForwardKinematics_d(self.Chain)
        self.dof = Chain.getNumOfJoints()
        self.adjv = DynamicsLib.AdjointSE3Vector(self.Chain.getNumOfJoints())
        print(self.dof)
        print('Debug 2')
        self.limits = DynamicsLib.LimitsVector(self.dof)
        
        self.Chain.getJointLimits(self.limits)
        print('Debug 3')
        self.CF = RegressorCostFunction.RegressorCostFunction_d(self.Chain)
        # quit(1)

        print('Debug 4')
        self.nf = [TrajModel.getFourierCoefficientNumber()] * self.dof

        self.velocity_limit = self.config['velLimit']
        self.acc_limit = self.config['accLimit']
        self.torque_limit = self.config['torqueLimit']
        print('Debug 5')
        self.wf_min = self.config['trajectoryPulseMin']
        self.wf_max = self.config['trajectoryPulseMax']
        self.wf_init = self.config['trajectoryPulseInit']
        self.TrajModel.setGlobalPulsation(self.wf_init)
        self.max_pos = self.config['max_pos']
        self.min_pos = self.config['min_pos']
        self.max_link_pos = self.config['max_link_pos']
        self.min_link_pos = self.config['min_link_pos']
        print('Debug 6')
        self.amin = self.bmin = self.config['trajectoryCoeffMin']
        self.amax = self.bmax = self.config['trajectoryCoeffMax']
        self.samples = int(ceil(self.TrajModel.getPeriodLength() * self.config['controlRate']))
        print('number of samples')
        print self.samples

        self.last_best_f_f1 = 0
        self.num_constraints = 0
        print('Debug 7')

        if(self.config['collision_constraints']):
            self.collision_constraints_index = self.num_constraints
            self.num_constraints += 1  # FK.number_of_collision_checks()
        if(self.config['joint_limit_constraint']):
            self.joint_limit_index = self.num_constraints
            self.num_constraints += self.dof * 2
        if(self.config['velocity_limit_constraint']):
            self.vel_limit_index = self.num_constraints
            self.num_constraints += self.dof
        if(self.config['acc_limit_constraint']):
            self.acc_limit_index = self.num_constraints
            self.num_constraints += self.dof
        if(self.config['torque_limit_constraint']):
            self.torque_limit_index = self.num_constraints
            self.num_constraints += self.dof
        if(self.config['pose_limit_constraints']):
            self.pose_constraint_index = self.num_constraints
            self.num_constraints += 3 * 2
        if(self.config['link_limit_constraints']):
            self.link_limit_constraints_index = self.num_constraints
            self.num_constraints += self.Chain.getNumOfJoints()*3*2
        print('Debug 8')
        np.seterr(all='raise')
        self.q_discrete = np.zeros((int(ceil(self.TrajModel.getPeriodLength()
                                        * self.config['controlRate'])), self.dof))
        self.qd_discrete = np.zeros((
            int(ceil(self.TrajModel.getPeriodLength()
                     * self.config['controlRate'])), self.dof))
        self.qdd_discrete = np.zeros((
            int(ceil(self.TrajModel.getPeriodLength()
                     * self.config['controlRate'])), self.dof))
        self.FK_discrete = np.zeros((
            int(ceil(self.TrajModel.getPeriodLength()
                     * self.config['controlRate'])), 3))

        
        self.link_poses = np.zeros((int(ceil(self.TrajModel.getPeriodLength()
                     * self.config['controlRate'])), 3*self.Chain.getNumOfJoints()))

        self.regressor = np.zeros(
            (self.dof * int(ceil(self.TrajModel.getPeriodLength() * self.config['controlRate'])), self.CF.getNumOfBaseParams()), dtype='float64')
        # import sys
        # print(sys.getsizeof(self.q_discrete)/1024)
        # print(sys.getsizeof(self.qd_discrete)/1024)
        # print(sys.getsizeof(self.qdd_discrete)/1024)
        # print(sys.getsizeof(self.FK_discrete)/1024)
        # print(sys.getsizeof(self.link_poses)/1024)
        # print(sys.getsizeof(self.regressor)/1024)
        # quit(-1)

    def vecToParams(self, x):
        # convert vector of all solution variables to separate parameter
        # variables
        wf = x[0]
        ab_len = self.dof * self.nf[0]
        a = np.array(
            np.split(np.array(x[1:1 + ab_len]), self.dof))
        b = np.array(np.split(
            np.array(x[1 + ab_len:1 + ab_len * 2]), self.dof))
        return wf, a, b

    def testBounds(self, x):
        # test variable bounds
        wf, a, b = self.vecToParams(x)
        wf_t = True
        try:
            a_t = np.all(a <= self.amax) and np.all(a >= self.amin)
            b_t = np.all(b <= self.bmax) and np.all(b >= self.bmin)
        except:
            print "A: ",a
            print "B: ",b
            self.comm.Finalize()
            exit(-1)
        res = wf_t and a_t and b_t
        return True

    def testConstraints(self, g):
        g = np.array(g)
        # start where collision constraints start
        if(self.config['collision_constraints']):
            c_s = self.collision_constraints_index

            res_c = (g[c_s] == 0)
            res = np.all(g[c_s+1:] <= self.config['minTolConstr'])
            return res and res_c
        else:
            res = np.all(g[:] <= self.config['minTolConstr'])
            return res

    def objectiveFunc(self, x2, test=False):
        # print "D1"
        self.iter_cnt += 1
        x = np.insert(x2, 0, self.wf_init, axis=0)
        # print x
        if not self.testBounds(x):
            print "fail Bounds:", self.mpi_rank
            f = 1000000
            g = [100.0] * self.num_constraints
            fail = 1.0
            return f, g, fail
        # print "D2"
        self.TrajModel.setFromParameters(np.array(x))
        trajectory_data = self.TrajModel.simulate(self.config['controlRate'])
        # print "D3"
        
        # print "D4"
        collision = False
        # print "D5"
        collision_output = self.CF.compute(trajectory_data, self.q_discrete, self.qd_discrete,
                        self.qdd_discrete, self.FK_discrete, self.regressor, collision)
        collision = collision_output[0]
        if(collision_output[1]!=collision):
            print "ERROR ON COLLISION!!"
            print collision_output
            print collision_output[1], " : ", collision
        # print "D6"
        # if(collision_output):
        #     print "collide python"
        #     print collision_output

        # print(self.regressor[self.dof*4:self.dof*4+self.dof,0:12*7].T)
        # print(q_discrete[4,:])
        # print(qd_discrete[4,:])
        # print(qdd_discrete[4,:])
        # quit(-1);
        # print (np.matrix(self.regressor).T * np.matrix(self.regressor)).shape
        # print np.linalg.eig((np.matrix(self.regressor).T *
        # np.matrix(self.regressor)))

        # input, &q_out, &qd_out, &qdd_out, &fk_pos_out, &regressor, &collision

        # f = np.linalg.norm(self.regressor) * np.linalg.norm(
        #     np.linalg.pinv(self.regressor))
        # print(self.regressor.shape)
        # print(self.regressor)
        # f = np.linalg.cond(self.regressor)#, p=-2)
        # np.set_printoptions(threshold=sys.maxsize)
        # print np.matrix(self.regressor).T * np.matrix(self.regressor)
        f = np.linalg.cond(
            (np.matrix(self.regressor).T * np.matrix(self.regressor)))
        # u,s,v = np.linalg.svd(self.regressor);
        # print f
        # u, s, vh = np.linalg.svd(self.regressor)
        # print s
        # f = s[0] / s[s.shape[0] - 1]
        f1 = 0
        g = [1e10] * self.num_constraints
        for n in range(self.dof):
            j = 0
            if self.config['joint_limit_constraint']:
                g[n+self.joint_limit_index] = self.limits[n][0] -\
                    np.min(self.q_discrete[:, n])
                g[self.dof +
                  n+self.joint_limit_index] = np.max(self.q_discrete[:, n]) - self.limits[n][1]
                j = j + 2

            if self.config['velocity_limit_constraint']:
                g[self.vel_limit_index + n] = np.max(
                    np.abs(self.qd_discrete[:, n])) - self.velocity_limit
                j = j + 1
        if self.config['acc_limit_constraint']:
            g[self.acc_limit_index:self.acc_limit_index + self.dof] = (np.max(
                np.abs(self.qdd_discrete[:, n])) - np.array(self.acc_limit))
            j = j + self.dof

        if self.config['pose_limit_constraints']:
            g[self.pose_constraint_index] = np.max(
                self.FK_discrete[:, 0]) - self.max_pos[0]
            g[self.pose_constraint_index +
              1] = np.max(self.FK_discrete[:, 1]) - self.max_pos[1]
            g[self.pose_constraint_index +
              2] = np.max(self.FK_discrete[:, 2]) - self.max_pos[2]
            g[self.pose_constraint_index + 3] = self.min_pos[0] -\
                np.min(self.FK_discrete[:, 0])
            g[self.pose_constraint_index +
              4] = self.min_pos[1] - np.min(self.FK_discrete[:, 1])
            g[self.pose_constraint_index +
              5] = self.min_pos[2] - np.min(self.FK_discrete[:, 2])
            # print 'min pos: ', np.min(FK_discrete[:, 2])
            # print 'min constr: ', self.min_pos[2] - np.min(FK_discrete[:, 2])
        if self.config['link_limit_constraints']:
            for i in range(self.samples):
                # q = trajectory_data.pos[:, i]
                # qd = trajectory_data.vel[:, i]
                self.Chain.update( trajectory_data.pos[:, i].copy(),trajectory_data.vel[:, i].copy())
                self.kin.getAdjointsVector(self.adjv,False)
                for i2 in range(self.Chain.getNumOfJoints()):
                    self.link_poses[i,3*i2:3*i2+3] = self.adjv[i2].getP().T
            for i in range(self.Chain.getNumOfJoints()):
                g[self.link_limit_constraints_index+6*i] = np.max(self.link_poses[:,3*i]) - self.max_link_pos[0,i]
                g[self.link_limit_constraints_index+1+6*i] = np.max(self.link_poses[:,3*i + 1]) - self.max_link_pos[1,i]
                g[self.link_limit_constraints_index+2+6*i] = np.max(self.link_poses[:,3*i + 2]) - self.max_link_pos[2,i]
                g[self.link_limit_constraints_index+3+6*i] = self.min_link_pos[0,i] - np.min(self.link_poses[:,3*i])
                g[self.link_limit_constraints_index+4+6*i] = self.min_link_pos[1,i] - np.min(self.link_poses[:,3*i + 1])
                g[self.link_limit_constraints_index+5+6*i] = self.min_link_pos[2,i] - np.min(self.link_poses[:,3*i + 2])

        if(self.config['collision_constraints']):
            g[self.collision_constraints_index] = collision
        # print g
        c = self.testConstraints(g)
        # print c
        if c and f < self.last_best_f:
            print 'Good'
            self.last_best_f = f
            self.last_best_f_f1 = f1
            self.last_best_sol = x
            self.last_best_sol2 = x2
        if ((self.iter_cnt - 1) % 100 == 0):
            print("objective function value: {} (last best: {} + {})".format(f,
                                                                             self.last_best_f - self.last_best_f_f1, self.last_best_f_f1))
            if self.config['joint_limit_constraint']:
                print("joint_limit_g: {}".format(g[self.joint_limit_index:self.joint_limit_index+self.dof]))
            if self.config['velocity_limit_constraint']:   
                print("velocity_limit_g: {}".format(g[self.vel_limit_index:self.vel_limit_index+self.dof]))
            if self.config['acc_limit_constraint']:
                print("acc_limit_constraint_g: {}".format(g[self.acc_limit_index:self.acc_limit_index+self.dof]))

            if self.config['pose_limit_constraints']:
                print("pose_limit_constraints_g: {}".format(g[self.pose_constraint_index:self.pose_constraint_index+6]))
            if self.config['link_limit_constraints']:
                print("link_limit_constraints: {}".format(g[self.link_limit_constraints_index:self.link_limit_constraints_index+6*self.Chain.getNumOfJoints()]))

            if(self.config['collision_constraints']):
                print("collision_constraints_g: {}".format(g[self.collision_constraints_index]))
        # c = True
        fail = 0.0
        return f, g, fail

    def addVarsAndConstraints(self, opt_prob):
        # type: (pyOpt.Optimization) -> None
        ''' add variables, define bounds '''

        # w_f -pulsation
        # opt_prob.addVar('wf', 'c', value=self.wf_init,
        #                 lower=self.wf_min, upper=self.wf_max)
        self.ainit = self.TrajModel.getFourierCoefficientsA()
        self.binit = self.TrajModel.getFourierCoefficientsB()
        # a, b -sin/cos params
        import random

        for j in range(self.nf[0]):
            for i in range(self.dof):
                if j == 0:
                    opt_prob.addVar('a{}_{}'.format(
                        i, j), 'c', value=self.config['trajectoryCoeffInit'][i], lower=self.amin, upper=self.amax)
                else:
                    opt_prob.addVar('a{}_{}'.format(
                        i, j), 'c', value=(random.uniform(self.amin, self.amax)) * 0, lower=self.amin, upper=self.amax)
        for j in range(self.nf[0]):
            for i in range(self.dof):
                opt_prob.addVar('b{}_{}'.format(
                    j, i), 'c', value=(random.uniform(self.bmin, self.bmax)) * 0, lower=self.bmin, upper=self.bmax)

        # add constraint vars (constraint functions are in obfunc)
        self.constraints_no = self.num_constraints
        if(self.config['collision_constraints']):
            opt_prob.addCon('g1',type='i')
            self.constraints_no= self.num_constraints-1

        opt_prob.addConGroup('g2', self.constraints_no,
                             type='i')

    def optimizeTrajectory(self):
        # Instanciate Optimization Problem
        opt_prob = pyOpt.Optimization(
            'Trajectory optimization', self.objectiveFunc, sens_type='FD')
        opt_prob.addObj('f')
        self.opt_prob = opt_prob

        self.addVarsAndConstraints(opt_prob)
        sol_vec = self.runOptimizer(opt_prob)
        # print sol_vec
        # sol_wf, sol_a, sol_b = self.vecToParams(sol_vec)
        self.TrajModel.setFromParameters(np.array(sol_vec))

        return self.TrajModel
