#!/bin/bash
for i in {0..10}; do
    mpiexec --use-hwthread-cpus -n 4 python ./OptimizeTrajectory.py ~/ProjectsInstall/PhD/cogimon_ws/install/share/gazebo/models/cogimon/kuka-lwr-3/model.urdf --output-file new_${i}.json
done
