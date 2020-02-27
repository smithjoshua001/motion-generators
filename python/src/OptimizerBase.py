try:
    import mpi4py
    mpi4py.rc.recv_mprobe = False
    from mpi4py import MPI
    comm = MPI.COMM_WORLD
    nprocs = comm.Get_size()
    parallel = nprocs > 1
except Exception as i:
    # print i
    parallel = False

import pyOpt
import random
import numpy as np


class OptimizerBase(object):
    def __init__(self, config):
        self.config = config
        self.comm = MPI.COMM_WORLD

        self.last_best_f = np.inf
        self.last_best_sol = np.array([])
        self.last_best_sol2 = np.array([])
        self.iter_cnt = 0
        self.last_g = None
        self.is_global = False
        self.local_iter_max = "(unknown)"

        self.parallel = parallel
        if(self.parallel):
            self.mpi_size = nprocs
            self.mpi_rank = comm.Get_rank()
            self.comm = comm
        else:
            self.mpi_size = 1
            self.mpi_rank = 0

    def testBounds(self, x):
        raise NotImplementedError

    def testConstraints(self, g):
        raise NotImplementedError

    def objectiveFunc(self, x, test=False):
        raise NotImplementedError

    def gather_solutions(self, all_update=False):
        # send best solutions to node 0
        if self.parallel:
            send_obj = [self.last_best_f, self.last_best_sol,
                        self.last_best_sol2, self.mpi_rank]
            received_objs = []
            print self.mpi_rank, ": Waiting for all gather"
            if all_update:
                received_objs = self.comm.allgather(send_obj)
            else:
                received_objs = self.comm.gather(send_obj,root=0)
            if(self.mpi_rank==0 or all_update):
                print self.mpi_rank, ": done all gather"
                for proc in range(0, self.mpi_size):
                    other_best_f, other_best_sol, other_best_sol2, rank = received_objs[proc]

                    # if (self.mpi_rank == 0 or self.last_best_f == float('Inf')) and other_best_f < self.last_best_f:
                    #     print('received better solution from {}: {} {}'.format(
                    #         rank, other_best_f, self.last_best_f))
                    #     self.last_best_f = other_best_f
                    #     self.last_best_sol = other_best_sol
                    #     self.last_best_sol2 = other_best_sol2
                    #     valid = True
                    # elif other_best_f < self.last_best_f and self.last_best_f == float('Inf'):
                    #     self.last_best_f = other_best_f
                    #     self.last_best_sol = other_best_sol
                    #     self.last_best_sol2 = other_best_sol2
                    #     valid = True
                    # elif other_best_f < self.last_best_f:
                    #     valid = True 
                    if other_best_f < self.last_best_f:
                        self.last_best_f = other_best_f
                        self.last_best_sol = other_best_sol
                        self.last_best_sol2 = other_best_sol2
                        valid = True

                if self.last_best_f == float('Inf'):
                    print("No solution found!")
                    self.comm.finalize()
                    exit(-1)

    def globalOptimizer(self, opt_prob, initial):
        # optimize using pyOpt (global)
        sr = random.SystemRandom()
        if self.config['globalSolver'] == 'NSGA2':
            if parallel:
                opt = pyOpt.NSGA2(pll_type='POA')  # genetic algorithm
            else:
                opt = pyOpt.NSGA2()
            if self.config['globalOptSize'] % 4:
                raise IOError(
                    "globalOptSize needs to be a multiple of 4 for NSGA2")
            # Population Size (a Multiple of 4)
            opt.setOption('PopSize', self.config['globalOptSize'])
            # Maximum Number of Generations
            opt.setOption('maxGen', self.config['globalOptIterations'])
            # Flag to Turn On Output to files (0-None, 1-Subset, 2-All)
            opt.setOption('PrintOut', 0)
            # Use Initial Solution Flag (0 -random population, 1 -use given solution)
            opt.setOption('xinit', 0)
            # Random Number Seed 0..1 (0 -Auto based on time clock)
            opt.setOption('seed', sr.random())
            # pCross_real  0.6   Probability of Crossover of Real Variable (0.6-1.0)
            # Probablity of Mutation of Real Variables (1/nreal)
            opt.setOption('pMut_real', 0.5)
            # eta_c 10.0  # Distribution Index for Crossover (5-20) must be > 0
            # eta_m 20.0  # Distribution Index for Mutation (5-50) must be > 0
            # pCross_bin   0.0   # Probability of Crossover of Binary Variable (0.6-1.0)
            # pMut_real   0.0   # Probability of Mutation of Binary Variables (1/nbits)
            self.iter_max = self.config['globalOptSize'] * \
                self.config['globalOptIterations']
        elif self.config['globalSolver'] == 'ALPSO':
            if parallel:
                # augmented lagrange particle swarm optimization
                opt = pyOpt.ALPSO(pll_type='SPM')
            else:
                opt = pyOpt.ALPSO()  # augmented lagrange particle swarm optimization
            opt.setOption('stopCriteria', 0)  # stop at max iters
            opt.setOption('dynInnerIter', 1)  # dynamic inner iter number
            opt.setOption('maxInnerIter', 5)
            opt.setOption('maxOuterIter',
                          self.config['globalOptIterations'])
            # opt.setOption('printInnerIters', 1)
            # opt.setOption('printOuterIters', 1)
            opt.setOption('SwarmSize', self.config['globalOptSize'])
            opt.setOption('xinit', 0)
            # (self.mpi_rank+1)/self.mpi_size)
            opt.setOption('seed', sr.random()*self.mpi_size)
            opt.setOption('itol', self.config['minTolConstr'])
            # opt.setOption('PrintOut', 0)
            opt.setOption('fileout', 0)
            # opt.setOption('vinit', 0.5)
            # opt.setOption('vmax', 1.0)

            opt.setOption('vcrazy', 1e-4)
            # TODO: how to properly limit max number of function calls?
            # no. func calls = (SwarmSize * inner) * outer + SwarmSize
            self.iter_max = opt.getOption('SwarmSize') * opt.getOption('maxInnerIter') * \
                opt.getOption('maxOuterIter') + opt.getOption('SwarmSize')
            self.iter_max = self.iter_max // self.mpi_size
        else:
            print("Solver {} not defined".format(
                self.config['globalSolver']))
            
            self.comm.finalize()
            exit(1)

        # run global optimization

        # try:
            # reuse history
        #  opt(opt_prob, store_hst=False, hot_start=True) #, xstart=initial)
        # except NameError:

        if self.config['verbose']:
            print('Running global optimization with {}'.format(
                self.config['globalSolver']))
        self.is_global = True
        import time
        # time.sleep(30)
        # try:
        opt(opt_prob, store_hst=False, xstart=np.matrix(initial))
        # except:
        #     print "GLOBAL FAIL?"
        if self.mpi_rank == 0:
            print(opt_prob.solution(0))
        print "GATHERING GLOBAL:", self.mpi_rank 
        self.gather_solutions(all_update=True)
        print "GATHERED GLOBAL:", self.mpi_rank 
        self.comm.barrier()

    def localOptimizer(self, opt_prob):
        # TODO: run local optimization for e.g. the three last best results (global solutions
            # could be more or less optimal within their local minima)

            # after using global optimization, refine solution with gradient based method init
            # optimizer (more or less local)
        if self.config['localSolver'] == 'SLSQP':
            # if parallel:
            #     opt2 = pyOpt.SLSQP(pll_type='POA')
            # else:
            opt2 = pyOpt.SLSQP()  # sequential least squares
            opt2.setOption('MAXIT', self.config['localOptIterations'])
            opt2.setOption('IPRINT',0)
            if self.config['verbose']:
                opt2.setOption('IPRINT',0)
        elif self.config['localSolver'] == 'IPOPT':
            opt2 = pyOpt.IPOPT()
            # mumps or hsl: ma27, ma57, ma77, ma86, ma97 or mkl: pardiso
            opt2.setOption('linear_solver', 'ma57')
            opt2.setOption('max_iter', self.config['localOptIterations'])
            if self.config['verbose']:
                opt2.setOption('print_level', 4)  # 0 none ... 5 max
            else:
                opt2.setOption('print_level', 0)  # 0 none ... 5 max
        elif self.config['localSolver'] == 'PSQP':
            opt2 = pyOpt.PSQP()
            # max iterations
            opt2.setOption('MIT', self.config['localOptIterations'])
            # opt2.setOption('MFV', ??) # max function evaluations
        elif self.config['localSolver'] == 'COBYLA':
            # if parallel:
            #     opt2 = pyOpt.COBYLA(pll_type='POA')
            # else:
            opt2 = pyOpt.COBYLA()
            # max iterations
            opt2.setOption('MAXFUN', self.config['localOptIterations'])
            opt2.setOption('RHOBEG', 0.01)  # initial step size
            if self.config['verbose']:
                opt2.setOption('IPRINT', 2)

        self.iter_max = self.local_iter_max
        #opt2.setOption('PrintOut', 0)
        # use best constrained solution from last run (might be better than what solver thinks)
        global_sol2 = self.last_best_sol2
        global_f = self.last_best_f
        global_sol = self.last_best_sol
        if len(self.last_best_sol2) > 0:
            for i in range(len(opt_prob.getVarSet())):
                opt_prob.getVar(i).value = self.last_best_sol2[i]

        if self.config['verbose']:
            print('Runing local optimization with {}'.format(
                self.config['localSolver']))
        self.is_global = False
        print "RUNNING LOCAL:", self.mpi_rank
        self.comm.barrier()
        try:
            # if self.config['localSolver'] in ['COBYLA', 'CONMIN']:
            #     opt2(opt_prob, store_hst=False)
            # else:
            # if parallel:
            #     opt2(opt_prob, sens_step=1e-6,
            #             sens_mode='pgc', store_hst=False)
            # else:
            opt2(opt_prob, sens_step=1e-6, store_hst=False)
        except:
            print "LOCAL FAILED REVERTING TO GLOBAL"
            self.last_best_sol2= global_sol2
            self.last_best_f = global_f
            self.last_best_sol = global_sol
        print "GATHERING SOLUTIONS"
        self.gather_solutions(all_update=False)
        print "FINISHED GATHERING SOLUTIONS"
        self.comm.barrier()

    def runOptimizer(self, opt_prob):
        # try:
        # type: (pyOpt.Optimization) -> np._ArrayLike[float]
        ''' call global followed by local optimizer, return solution '''

        import pyOpt
        # import deep_copy
        # opt_prob2 = deep_copy(opt_prob)

        initial = [v.value for v in list(opt_prob.getVarSet().values())]
        inittemp = []
        for i in range(len(opt_prob.getVarSet())):
            inittemp.append(opt_prob.getVar(i).value)
        self.opt_prob = opt_prob
        import time
        self.iter_cnt = 0
        if self.config['useGlobalOptimization']:
            self.globalOptimizer(opt_prob, inittemp)
            # pyOpt local
        
        if(self.mpi_rank==0):
            self.TrajModel.setFromParameters(np.array(self.last_best_sol))
            self.TrajModel.saveToJSON("traj-global.json")
        # self.opt_prob = opt_prob2
        if self.config['useLocalOptimization']:
            # print("Runnning local gradient based solver")
            self.localOptimizer(opt_prob)

        if self.mpi_rank == 0:
            sol = opt_prob.solution(1)
            # print(sol)
            # print opt_prob.getSolSet()
            # sol_vec = np.array([sol.getVar(x).value for x in range(0,len(sol.getVarSet()))])
            if len(self.last_best_sol2) > 0:
                print(
                    "using last best constrained solution instead of given solver solution.")
                # print(sol)
                # print("testing final solution")
                # self.iter_cnt = 0
                print self.last_best_sol2
                self.objectiveFunc(self.last_best_sol2, test=True)
                # print("\n")
                return self.last_best_sol
            else:
                print("No feasible solution found!")  
                MPI.Finalize()
                exit(-1)
        else:
            # # parallel sub-processes, close
            print "CLOSING 1"
            exit(0)
        # finally:
        #     # if self.mpi_rank == 0:
        #     print "CLOSING 2"
        #     MPI.Finalize()
        #     exit(0)