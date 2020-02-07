#!/usr/bin/python
import DynamicsLib
import numpy as np
import ModifiedFourierTrajectory
import OptimizerBase
import MFTOptimizerOptimized
import MFTOptimizer
import argparse
try:
    from mpi4py import MPI
    comm = MPI.COMM_WORLD
    nprocs = comm.Get_size()
    parallel = nprocs > 1
except Exception as i:
    # print i
    parallel = False


from math import ceil
np.set_printoptions(precision=3, suppress=True, linewidth=200)


def main():
    parser = argparse.ArgumentParser(
        description='Optimize a modified fourier trajectory with respect to the condition number. Considering constraints.')
    parser.add_argument('urdf_file', metavar='file_name', type=str,
                        nargs=1, help='URDF file of the robot for structure information')
    parser.add_argument('--output-file', dest='of',
                        help='output file', default='traj.json')
    args = parser.parse_args()
    print('main debug 1')
    chain = DynamicsLib.Chain_d.initChain(args.urdf_file[0])
    print('main debug 2')
    trajModel = ModifiedFourierTrajectory.ModifiedFourierTrajectory_d(
        chain.getNumOfJoints(), 12)
    print(chain.getNumOfJoints())

    # useGlobalOptimization
    # globalSolver NSGA2 ALPSO
    # globalOptSize
    # globalOptIterations
    # useLocalOptimization
    # localSolver SLSQP IPOPT PSQP COBYLA
    # verbose
    # localOptIterations
    # velLimit
    # torqueLimit
    # trajectoryPulseMin
    # trajectoryPulseMax
    # trajectoryPulseInit
    # max_pos
    # min_pos
    # trajectoryCoeffMin
    # trajectoryCoeffMax
    # joint_limit_constraint
    # velocity_limit_constraint
    # torque_limit_constraint
    # pose_limit_constraints
    # controlRate
    # minTolConstr
    # 'collision_constraints': True
    link_pose_limits_max = np.matrix([[1.8,1.8,1.8,1.8,1.8,1.8,1.8],
                                      [1.8,1.8,1.8,1.8,1.8,1.8,1.8],
                                      [1.8,1.8,1.8,1.8,1.8,1.8,1.8]])
    link_pose_limits_min = np.matrix([[-1.8,-1.8,-1.8,-1.8,-1.8,-1.8,-1.8],
                                      [-1.8,-1.8,-1.8,-1.8,-1.8,-1.8,-1.8],
                                      [0,0.15,0.3,0.3,0.3,0.3,0.3]])                                  

    config = {'useGlobalOptimization': True, 'globalSolver': 'ALPSO', 'globalOptSize': 40,
              'globalOptIterations': 50, 'useLocalOptimization': True, 'localSolver': 'SLSQP',
    'localOptIterations': 500, 'minTolConstr': 1e-5, 'trajectoryPulseInit': 0.4,#0.628,#0.4,  # 0.209,
              'trajectoryPulseMin': 0.3, 'trajectoryPulseMax': 0.5, 'trajectoryCoeffInit': [0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01,0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01],
              'trajectoryCoeffMin': -0.07, 'trajectoryCoeffMax': 0.07, 'joint_limit_constraint': 1, 'velocity_limit_constraint': 1, 'velLimit': 1.5,
              'torque_limit_constraint': 0, 'torqueLimit': 90, 'pose_limit_constraints': 1, 'controlRate': 100, 'collision_constraints': True, 'max_pos': np.array([1.8, 1.8, 2]), 'min_pos': np.array([-1.8, -1.8, 0.3]),
              'verbose': 1, 'acc_limit_constraint': 0, 'accLimit': [16.5, 8.25, 13.75, 13.75, 16.5, 22, 22], 'link_limit_constraints': 1, 'max_link_pos':link_pose_limits_max,'min_link_pos':link_pose_limits_min}

    # config = {
    #     'useGlobalOptimization': True, 'globalSolver': 'ALPSO', 'globalOptSize': 150,
    #           'globalOptIterations': 25, 'useLocalOptimization': True, 'localSolver': 'COBYLA',
    #           'localOptIterations': 10000, 'minTolConstr': 0.0001, 'trajectoryPulseInit': 0.1,  # 0.209,
    #           'trajectoryPulseMin': 0.04, 'trajectoryPulseMax': 0.9, 'trajectoryCoeffInit': [0.2, 0.1],
    #           'trajectoryCoeffMin': -0.4, 'trajectoryCoeffMax': 0.4, 'joint_limit_constraint': 1, 'velocity_limit_constraint': 0, 'velLimit': 2.5,
    #           'torque_limit_constraint': 0, 'torqueLimit': 87, 'pose_limit_constraints': 0, 'controlRate': 100, 'collision_constraints': True, 'max_pos': np.array([6, 6, 6]), 'min_pos': np.array([-6, -6, -0.2]),
    #           'verbose': 1, 'acc_limit_constraint': 0, 'accLimit': [16.5, 8.25]}

    optim = MFTOptimizerOptimized.ModifiedFourierTrajectoryOptimzerOptim(
        trajModel, chain, config)
    # optim = MFTOptimizer.ModifiedFourierTrajectoryOptimzer(
    #     trajModel, chain, config)
    trajModel = optim.optimizeTrajectory()
    if comm.Get_rank() != 0:
        print "CLOSE 3"
        exit(0)

    trajModel.saveToJSON(args.of)


if __name__ == "__main__":
    main()
