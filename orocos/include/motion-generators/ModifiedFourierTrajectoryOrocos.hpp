#pragma once

#include <motion-generators/ModifiedFourierTrajectory.hpp>
#include <motion-generators/JointTrajectoryOrocos.hpp>

class ModifiedFourierTrajectoryOrocos : public RTT::TaskContext, public JointTrajectoryOrocos {
public:
    ModifiedFourierTrajectoryOrocos(std::string name) : RTT::TaskContext(name),
        JointTrajectoryOrocos(this, std::make_shared < ModifiedFourierTrajectory<float> >()) {}
};
