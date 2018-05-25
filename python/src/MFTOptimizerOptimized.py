import DynamicsLib
import numpy as np
import ModifiedFourierTrajectory
import OptimizerBase
import pyOpt
import RegressorCostFunction

from math import ceil


class ModifiedFourierTrajectoryOptimzerOptim(OptimizerBase.OptimizerBase):
    def __init__(self, TrajModel, Chain, config):
        super(ModifiedFourierTrajectoryOptimzerOptim, self).__init__(config)
        self.TrajModel = TrajModel
        self.Chain = Chain
        self.limits = DynamicsLib.LimitsVector(7)
        self.Chain.getJointLimits(self.limits)
        self.CF = RegressorCostFunction.RegressorCostFunction_d(self.Chain)

        self.dof = Chain.getNumOfJoints()
        self.nf = [TrajModel.getFourierCoefficientNumber()] * self.dof

        self.velocity_limit = self.config['velLimit']
        self.acc_limit = self.config['accLimit']
        self.torque_limit = self.config['torqueLimit']

        self.wf_min = self.config['trajectoryPulseMin']
        self.wf_max = self.config['trajectoryPulseMax']
        self.wf_init = self.config['trajectoryPulseInit']

        self.max_pos = self.config['max_pos']
        self.min_pos = self.config['min_pos']

        self.amin = self.bmin = self.config['trajectoryCoeffMin']
        self.amax = self.bmax = self.config['trajectoryCoeffMax']

        self.last_best_f_f1 = 0
        self.num_constraints = 0
        if(self.config['joint_limit_constraint']):
            self.num_constraints += self.dof*2
        if(self.config['velocity_limit_constraint']):
            self.num_constraints += self.dof
        if(self.config['acc_limit_constraint']):
            self.num_constraints += self.dof
        if(self.config['torque_limit_constraint']):
            self.num_constraints += self.dof
        if(self.config['pose_limit_constraints']):
            self.pose_constraint_index = self.num_constraints
            self.num_constraints += 3*2
        if(self.config['collision_constraints']):
            self.collision_constraints_index = self.num_constraints
            self.num_constraints += 1  # FK.number_of_collision_checks()

    def vecToParams(self, x):
        # convert vector of all solution variables to separate parameter variables
        wf = x[0]
        ab_len = self.dof*self.nf[0]
        a = np.array(
            np.split(np.array(x[1:1+ab_len]), self.dof))
        b = np.array(np.split(
            np.array(x[1+ab_len:1+ab_len*2]), self.dof))
        return wf, a, b

    def testBounds(self, x):
        # test variable bounds
        wf, a, b = self.vecToParams(x)
        wf_t = True
        a_t = np.all(a <= self.amax) and np.all(a >= self.amin)
        b_t = np.all(b <= self.bmax) and np.all(b >= self.bmin)
        res = wf_t and a_t and b_t
        return res

    def testConstraints(self, g):
        g = np.array(g)
        # start where collision constraints start
        c_s = self.collision_constraints_index

        res = np.all(g[:c_s] <= self.config['minTolConstr'])
        res_c = g[c_s:] == 0
        return res and res_c

    def objectiveFunc(self, x2, test=False):
        self.iter_cnt += 1
        x = np.insert(x2, 0, self.wf_init, axis=0)
        # print x
        if not self.testBounds(x):
            f = 10000.0
            g = [10.0]*self.num_constraints
            fail = 1.0
            return f, g, fail

        self.TrajModel.setFromParameters(np.array(x))
        trajectory_data = self.TrajModel.simulate(self.config['controlRate'])

        q_discrete = np.zeros((int(ceil(self.TrajModel.getPeriodLength()
                                        * self.config['controlRate'])), self.dof))
        qd_discrete = np.zeros((
            int(ceil(self.TrajModel.getPeriodLength()
                     * self.config['controlRate'])), self.dof))

        qdd_discrete = np.zeros((
            int(ceil(self.TrajModel.getPeriodLength()
                     * self.config['controlRate'])), self.dof))
        FK_discrete = np.zeros((
            int(ceil(self.TrajModel.getPeriodLength()
                     * self.config['controlRate'])), 3))
        collision = False
        self.regressor = np.zeros(
            (self.dof * int(ceil(self.TrajModel.getPeriodLength() * self.config['controlRate'])), self.CF.getNumOfBaseParams()), dtype='float64')
        self.CF.compute(trajectory_data, q_discrete, qd_discrete,
                        qdd_discrete, FK_discrete, self.regressor, collision)
        # input, &q_out, &qd_out, &qdd_out, &fk_pos_out, &regressor, &collision

        f = np.linalg.cond(self.regressor)
        f1 = 0
        g = [1e10]*self.num_constraints
        for n in range(self.dof):
            j = 0
            if self.config['joint_limit_constraint']:
                g[n] = self.limits[n][0] -\
                    np.min(q_discrete[:, n])
                g[self.dof +
                  n] = np.max(q_discrete[:, n]) - self.limits[n][1]
                j = j+2

            if self.config['velocity_limit_constraint']:
                g[j*self.dof+n] = np.max(
                    np.abs(qd_discrete[:, n])) - self.velocity_limit
                j = j+1

        if self.config['acc_limit_constraint']:
            g[j*self.dof:j*self.dof+self.dof] = (np.max(
                np.abs(qdd_discrete[:, n])) - np.array(self.acc_limit))
            j = j+self.dof

        if self.config['pose_limit_constraints']:
            g[self.pose_constraint_index] = np.max(
                FK_discrete[:, 0]) - self.max_pos[0]
            g[self.pose_constraint_index +
              1] = np.max(FK_discrete[:, 1]) - self.max_pos[1]
            g[self.pose_constraint_index +
              2] = np.max(FK_discrete[:, 2]) - self.max_pos[2]
            g[self.pose_constraint_index+3] = self.min_pos[0] -\
                np.min(FK_discrete[:, 0])
            g[self.pose_constraint_index +
              4] = self.min_pos[1] - np.min(FK_discrete[:, 1])
            g[self.pose_constraint_index +
              5] = self.min_pos[2] - np.min(FK_discrete[:, 2])
            # print 'min pos: ', np.min(FK_discrete[:, 2])
            # print 'min constr: ', self.min_pos[2] - np.min(FK_discrete[:, 2])

        if(self.config['collision_constraints']):
            g[self.collision_constraints_index] = collision

        c = self.testConstraints(g)
        if c and f < self.last_best_f:
            self.last_best_f = f
            self.last_best_f_f1 = f1
            self.last_best_sol = x
            self.last_best_sol2 = x2
        if ((self.iter_cnt-1) % 100 == 0):
            print("objective function value: {} (last best: {} + {})".format(f,
                                                                             self.last_best_f-self.last_best_f_f1, self.last_best_f_f1))
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
                        i, j), 'c', value=(random.uniform(self.amin, self.amax))*0, lower=self.amin, upper=self.amax)
        for j in range(self.nf[0]):
            for i in range(self.dof):
                opt_prob.addVar('b{}_{}'.format(
                    j, i), 'c', value=(random.uniform(self.bmin, self.bmax))*0, lower=self.bmin, upper=self.bmax)

        # add constraint vars (constraint functions are in obfunc)
        opt_prob.addConGroup('g', self.num_constraints,
                             type='i')

    def optimizeTrajectory(self):
        # Instanciate Optimization Problem
        opt_prob = pyOpt.Optimization(
            'Trajectory optimization', self.objectiveFunc, sens_type='CS')
        opt_prob.addObj('f')
        self.opt_prob = opt_prob

        self.addVarsAndConstraints(opt_prob)
        sol_vec = self.runOptimizer(opt_prob)
        print sol_vec
        # sol_wf, sol_a, sol_b = self.vecToParams(sol_vec)
        self.TrajModel.setFromParameters(np.array(sol_vec))

        return self.TrajModel
